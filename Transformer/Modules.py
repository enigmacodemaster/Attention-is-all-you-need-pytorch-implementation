import torch
import torch.nn as nn
import torch.nn.functional as F
import math

class ScaledDotProductAttention(nn.Module):
    ''' Scaled Dot-Product Attention '''

    def __init__(self, d_tensor, attn_dropout=0.1):
        super().__init__()
        self.scale = math.sqrt(d_tensor)
        self.dropout = nn.Dropout(attn_dropout)

    def forward(self, q, k, v, mask=None):
        # calculate the scaled dot product attention
        attn = torch.matmul(q, k.transpose(2, 3)) / self.scale # 最后两个维度相乘，所以进行张量最后两个维度的转置

        if mask is not None:
            attn = attn.masked_fill(mask == 0, float('-inf'))

        attn = self.dropout(F.softmax(attn, dim=-1)) # 在最后一个维度上计算
        output = torch.matmul(attn, v)

        return output, attn


class MultiHeadAttention(nn.Module):
    ''' Multi-Head Attention module '''

    def __init__(self, n_head, d_model, d_k, d_v, dropout=0.1):
        super().__init__()

        self.n_head = n_head
        self.d_k = d_k
        self.d_v = d_v

        self.w_qs = nn.Linear(d_model, n_head * d_k, bias=False)
        self.w_ks = nn.Linear(d_model, n_head * d_k, bias=False)
        self.w_vs = nn.Linear(d_model, n_head * d_v, bias=False)
        self.fc = nn.Linear(n_head * d_v, d_model, bias=False)

        self.attention = ScaledDotProductAttention(d_tensor=d_k)

        self.dropout = nn.Dropout(dropout)
        self.layer_norm = nn.LayerNorm(d_model, eps=1e-6)


    def forward(self, q, k, v, mask=None):

        d_k, d_v, n_head = self.d_k, self.d_v, self.n_head
        sz_b, len_q, len_k, len_v = q.size(0), q.size(1), k.size(1), v.size(1)

        residual = q

        # Pass through the pre-attention projection: b x lq x (n*dv)
        # Separate different heads: b x lq x n x dv
        # Transpose for attention dot product: b x n x lq x dv
        q = self.w_qs(q).view(sz_b, len_q, n_head, d_k).transpose(1, 2)
        k = self.w_ks(k).view(sz_b, len_k, n_head, d_k).transpose(1, 2)
        v = self.w_vs(v).view(sz_b, len_v, n_head, d_v).transpose(1, 2)


        if mask is not None:
            mask = mask.unsqueeze(1)   # For head axis broadcasting.

        q, attn = self.attention(q, k, v, mask=mask)

        # Transpose to move the head dimension back: b x lq x n x dv
        # Combine the last two dimensions to concatenate all the heads together: b x lq x (n*dv)
        q = q.transpose(1, 2).contiguous().view(sz_b, len_q, -1)
        q = self.dropout(self.fc(q))
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
d_model: embedding vector dimension
max_len: max sequence length model can handel
device: computing, cpu or cuda
'''
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_len=200, device='cpu'):
        super(PositionalEncoding, self).__init__()
        self.pos_table = self._get_sinusoid_encoding_table(max_len, d_model, device)
        
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