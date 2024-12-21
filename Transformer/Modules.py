import torch
import torch.nn as nn
import torch.nn.functional as F
import math
import logging

'''
注意力机制的主要操作包括：
1. 计算注意力权重
attn = softmax(QK^T / \sqrt{d_k} ) # attn 维度 -> (batch_size, n_head, len_q, len_k)
2. 将注意力权重应用到值(v)上，得到最终的输出
output = attn x V 
通常情况下，len_k 和 len_v 是相同的（因为 K 和 V 来自相同的输入），即 len_k = len_v。因此，v 的实际形状可以视为 (batch_size, n_head, len_k, d_v)。
'''


# 缩放点积 注意力
# 关键词: 点积
class ScaledDotProductAttention(nn.Module):
    ''' Scaled Dot-Product Attention '''
    # d_tensor: 输入特征维度，一般是embedding维度，也是Q或者K的维度
    # self.scale: 缩放因子，为特征维度的平方根，为了防止点积结果过大，导致softmax函数梯度消失
    def __init__(self, d_tensor, attn_dropout=0.1):
        super().__init__()
        self.scale = math.sqrt(d_tensor)
        self.dropout = nn.Dropout(attn_dropout) # 权重上的过拟合

    def forward(self, q, k, v, mask=None):
        # calculate the scaled dot product attention
        # 点积注意力得分： torch.matmul(q, k.transpose(2, 3))
        # 其中k的最后两个维度(len_k, d_k)被转置为(d_k, len_k)，然后与q相乘，结果为(batch_size, n_head, len_q, len_k)。
        attn = torch.matmul(q, k.transpose(2, 3)) / self.scale
        logging.info("scaled dot product attention shape: %s", attn.shape)
        # 应用掩码（可选）： 使用masked_fill将被遮蔽的位置设置为负无穷，确保这些位置在Softmax后权重接近于零
        if mask is not None:
            attn = attn.masked_fill(mask == 0, float('-inf'))

        attn = self.dropout(F.softmax(attn, dim=-1)) # 在最后一个维度上计算
        output = torch.matmul(attn, v) # (batch_size, n_head, len_q, d_v)
        logging.info("scaled dot product output shape: %s", output.shape)
        return output, attn

# 多头注意力：整体过程步骤总结
# - 线性变换和分头： embedding的输入经过线性投影后拆分成多个头。
# - 独立注意力计算： 每个头独立计算注意力。
# - 合并头输出： 将各头的输出拼接起来，并经过线性变换恢复原始维度。
# - 残差连接和归一化： 保持梯度流动和稳定训练。

class MultiHeadAttention(nn.Module):
    ''' Multi-Head Attention module '''
    # n_head: 注意力头的数量
    # d_model: embedding size
    # d_k: 每个头的键/查询维度
    # d_v: 每个头的值查询维度
    # 一般来说 d_k == d_v，可以不一样
    def __init__(self, n_head, d_model, d_k, d_v, dropout=0.1):
        super().__init__()

        self.n_head = n_head
        self.d_k = d_k
        self.d_v = d_v
        # w_qs, w_ks, w_vs: 线性变换层
        # 用于将输入的d_model维度投影到n_head * d_k或n_head * d_v, 这些层没有偏置项
        self.w_qs = nn.Linear(d_model, n_head * d_k, bias=False)
        self.w_ks = nn.Linear(d_model, n_head * d_k, bias=False)
        self.w_vs = nn.Linear(d_model, n_head * d_v, bias=False)
        # fc: 最终的线性变换层，将多头的输出拼接起来回到d_model维度
        self.fc = nn.Linear(n_head * d_v, d_model, bias=False)

        self.attention = ScaledDotProductAttention(d_tensor=d_k)
        # 
        self.dropout = nn.Dropout(dropout)
        self.layer_norm = nn.LayerNorm(d_model, eps=1e-6)


    def forward(self, q, k, v, mask=None):
        d_k, d_v, n_head = self.d_k, self.d_v, self.n_head
        sz_b, len_q, len_k, len_v = q.size(0), q.size(1), k.size(1), v.size(1) # batch_size, q的序列长度，k的序列长度, v的序列长度
        # 残差是query, 保存
        residual = q

        # Pass through the pre-attention projection: b x lq x (n*dv)
        # Separate different heads: b x lq x n x dv
        # Transpose for attention dot product: b x n x lq x dv
        # 通过view和transpose将投影后的q, k, v拆分成多个头，形状变为 (batch_size, n_head, len_q/k/v, d_k/d_v)
        # 多头关注机制需要：每个头独立处理输入序列，因此必须分拆输入数据以便每个头能在自己的子空间内进行独立计算
        # 感知多样的特征：通过将 v 张量变为 (sz_b, n_head, len_v, d_v) 形状，使每个注意力头能够感知和处理不同的特征，使得模型能更好地捕捉输入序列中的多样性和复杂性
        q = self.w_qs(q).view(sz_b, len_q, n_head, d_k).transpose(1, 2)
        k = self.w_ks(k).view(sz_b, len_k, n_head, d_k).transpose(1, 2)
        v = self.w_vs(v).view(sz_b, len_v, n_head, d_v).transpose(1, 2)


        if mask is not None:
            mask = mask.unsqueeze(1)   # For head axis broadcasting.
        # 将拆分后的q, k, v输入到ScaledDotProductAttention中，得到多头的注意力输出和权重
        q, attn = self.attention(q, k, v, mask=mask)

        # Transpose to move the head dimension back: b x lq x n x dv
        # 将多头的输出通过transpose和view重新合并，形状恢复为 (batch_size, len_q, n_head * d_v)
        q = q.transpose(1, 2).contiguous().view(sz_b, len_q, -1)
        # 使用fc层将合并后的多头输出投影回d_model维度，并应用Dropout
        q = self.dropout(self.fc(q))
        # 将残差residual加到输出上，然后进行层归一化
        q += residual

        q = self.layer_norm(q)

        return q, attn


class PositionwiseFeedForward(nn.Module):
    ''' A two-feed-forward-layer module '''

    def __init__(self, d_in, d_hid, dropout=0.1):
        super().__init__()
        self.w_1 = nn.Linear(d_in, d_hid) # position-wise
        self.w_2 = nn.Linear(d_hid, d_in) # position-wise
        self.layer_norm = nn.LayerNorm(d_in, eps=1e-6)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):

        residual = x

        x = self.w_2(F.relu(self.w_1(x)))
        x = self.dropout(x)
        x += residual

        x = self.layer_norm(x)

        return x

'''
positional encoding method
d_model: 嵌入向量维度
max_len: 模型能够处理的最大序列长度
device: computing, cpu or cuda
给输入序列添加位置信息, transformer不自带位置编码能力
'''
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, n_position=200, device='cpu'):
        super(PositionalEncoding, self).__init__()
        self.pos_table = self._get_sinusoid_encoding_table(n_position, d_model, device)
        
    def _get_sinusoid_encoding_table(self, max_seq_len, d_hid, device):
        ''' Sinusoid position encoding table '''
        position = torch.arange(0, max_seq_len, device=device).float().unsqueeze(1) # 生成位置索引，变为浮点数，并增加一个维度便于广播
        div_term = torch.exp(torch.arange(0, d_hid, 2, device=device).float() * -(torch.log(torch.tensor(10000.0)) / d_hid)) # 每个索引位置的缩放因子
        
        sinusoid_table = torch.zeros(max_seq_len, d_hid, device=device) # 创建一个矩阵，用于存储位置编码
        sinusoid_table[:, 0::2] = torch.sin(position * div_term) # 偶数填充正弦值
        sinusoid_table[:, 1::2] = torch.cos(position * div_term) # 奇数填充余弦值

        return sinusoid_table.unsqueeze(0)

    def forward(self, x):
        return x + self.pos_table[:, :x.size(1)].clone().detach()
    
# 输入序列 -> 嵌入层 + 位置编码 -> 多头注意力 -> 残差连接 + 层归一化 -> 前馈网络 -> 残差连接 + 层归一化 -> 输出
# 并行计算： 多头注意力使得模型能够并行学习不同的表示子空间，增强了建模能力。
# 位置编码： 通过引入固定的或可训练的位置编码，使得模型能够利用序列中元素的位置信息。
# 规范化与正则化： 残差连接、层归一化和Dropout帮助模型在训练深层网络时保持稳定性和泛化能力。

