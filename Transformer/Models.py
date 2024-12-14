import torch
import torch.nn as nn
import numpy as np
from Layers import EncoderLayer, DecoderLayer
from Modules import PositionalEncoding
import logging

# 生成填充mask，用来标识输入序列中哪些位置是实际数据，哪些是填充的空位。
# 这样模型在计算时，就可以忽略那些填充的部分
def get_pad_mask(seq, pad_idx):
    return (seq != pad_idx).unsqueeze(-2)

# 生成一种后续不可见的掩码，确保编码器在生成每个词的时候只能看见之前的已经生成的词，而不能看到之后的词
def get_subsequent_mask(seq):
    ''' For masking out the subsequent info. '''
    sz_b, len_s = seq.size()
    subsequent_mask = (1 - torch.triu(
        torch.ones((1, len_s, len_s), device=seq.device), diagonal=1)).bool()
    logging.info("debug info %s", subsequent_mask)
    return subsequent_mask

# 处理输入序列信息
class Encoder(nn.Module):
    ''' A encoder model with self attention mechanism. '''
    # 初始化编码器
    def __init__(
            self, n_src_vocab, d_word_vec, n_layers, n_head, d_k, d_v,
            d_model, d_inner, pad_idx, dropout=0.1, n_position=200, scale_emb=False):

        super().__init__()
        # src_word_emb: 将词ID转换为词向量
        self.src_word_emb = nn.Embedding(n_src_vocab, d_word_vec, padding_idx=pad_idx)
        # 因为transformer 不像RNN那样天然的有顺序处理能力，所以需要给输入的词向量添加一个位置编码，来高速模型每个词的位置在句子中的什么位置
        self.position_enc = PositionalEncoding(d_word_vec, n_position=n_position) # 添加位置信息，添加序列信息
        self.dropout = nn.Dropout(p=dropout)
        # 多个编码器层的堆叠，实际编码器工作在这个部分
        self.layer_stack = nn.ModuleList([
            EncoderLayer(d_model, d_inner, n_head, d_k, d_v, dropout=dropout)
            for _ in range(n_layers)])
        # 让训练更稳定，归一化技术
        self.layer_norm = nn.LayerNorm(d_model, eps=1e-6)
        # 是否对词向量进行缩放
        self.scale_emb = scale_emb
        # 隐藏层维度
        self.d_model = d_model

    # 前向传播，输入源语言序列和掩码
    def forward(self, src_seq, src_mask, return_attns=False):
        logging.info("EncoderMsg - src_word: %s", src_seq.shape)
        logging.info("EncoderMsg - src_mask: %s", src_mask.shape)
        enc_slf_attn_list = []

        # -- Forward
        enc_output = self.src_word_emb(src_seq)
        logging.info("EncoderMsg - src_word after word embedding: %s", enc_output.shape)
        if self.scale_emb:
            logging.info("EncoderMsg - scale emb enabled!")
            enc_output *= self.d_model ** 0.5
        enc_output = self.dropout(self.position_enc(enc_output))
        logging.info("EncoderMsg - embeddings after position_enc: %s", enc_output.shape)
        enc_output = self.layer_norm(enc_output)
        logging.info("EncoderMsg - embeddings after layer_norm: %s", enc_output.shape)

        for idx, enc_layer in enumerate(self.layer_stack):
            enc_output, enc_slf_attn = enc_layer(enc_output, slf_attn_mask=src_mask)
            logging.info("EncoderMsg - embeddings after %s encoder layer %s", idx, enc_output.shape)
            enc_slf_attn_list += [enc_slf_attn] if return_attns else []

        if return_attns:
            return enc_output, enc_slf_attn_list
        return enc_output,

# 将编码结果和之前已经生成的目标词一起处理，生成下一个目标词
# 
class Decoder(nn.Module):
    ''' A decoder model with self attention mechanism. '''

    def __init__(
            self, n_trg_vocab, d_word_vec, n_layers, n_head, d_k, d_v,
            d_model, d_inner, pad_idx, n_position=200, dropout=0.1, scale_emb=False):

        super().__init__()

        self.trg_word_emb = nn.Embedding(n_trg_vocab, d_word_vec, padding_idx=pad_idx)
        self.position_enc = PositionalEncoding(d_word_vec, n_position=n_position)
        self.dropout = nn.Dropout(p=dropout)
        self.layer_stack = nn.ModuleList([
            DecoderLayer(d_model, d_inner, n_head, d_k, d_v, dropout=dropout)
            for _ in range(n_layers)])
        self.layer_norm = nn.LayerNorm(d_model, eps=1e-6)
        self.scale_emb = scale_emb
        self.d_model = d_model

    # trg_seq: 目标语言序列
    # trg_mask: 掩码，包含后续不可见的部分
    # 对目标词进行嵌入
    def forward(self, trg_seq, trg_mask, encoder_output, src_mask, return_attns=False):
        logging.info("DecoderMsg - trg_seq: %s", trg_seq.shape)
        logging.info("DecoderMsg - trg_mask: %s", trg_mask.shape)
        logging.info("DecoderMsg - encoder_output: %s", encoder_output.shape)
        
        dec_slf_attn_list, dec_enc_attn_list = [], []

        # -- Forward
        dec_output = self.trg_word_emb(trg_seq)
        logging.info("DecoderMsg - target word embedding: %s", dec_output.shape)
        
        if self.scale_emb:
            dec_output *= self.d_model ** 0.5
        dec_output = self.dropout(self.position_enc(dec_output))
        dec_output = self.layer_norm(dec_output)

        for idx, dec_layer in enumerate(self.layer_stack):
            dec_output, dec_slf_attn, dec_enc_attn = dec_layer(
                dec_output, encoder_output, slf_attn_mask=trg_mask, dec_enc_attn_mask=src_mask)
            logging.info("DecoderMsg - embedding after %s layer %s", idx, dec_output.shape)
            dec_slf_attn_list += [dec_slf_attn] if return_attns else []
            dec_enc_attn_list += [dec_enc_attn] if return_attns else []

        if return_attns:
            return dec_output, dec_slf_attn_list, dec_enc_attn_list
        return dec_output,


class Transformer(nn.Module):
    ''' A sequence to sequence model with attention mechanism. '''
    # n_src_vocab: 源词汇表大小
    # n_trg_vocab: 目标词汇表大小
    # src_pad_idx: 源语言填充索引
    # trg_pad_idx: 目标语言填充索引
    # 
    def __init__(
            self, n_src_vocab, n_trg_vocab, src_pad_idx, trg_pad_idx,
            d_word_vec=512, d_model=512, d_inner=2048,
            n_layers=6, n_head=8, d_k=64, d_v=64, dropout=0.1, n_position=200,
            trg_emb_prj_weight_sharing=True, emb_src_trg_weight_sharing=True,
            scale_emb_or_prj='prj'):

        super().__init__()

        self.src_pad_idx, self.trg_pad_idx = src_pad_idx, trg_pad_idx

        # In section 3.4 of paper "Attention Is All You Need", there is such detail:
        # "In our model, we share the same weight matrix between the two
        # embedding layers and the pre-softmax linear transformation...
        # In the embedding layers, we multiply those weights by \sqrt{d_model}".
        #
        # Options here:
        #   'emb': multiply \sqrt{d_model} to embedding output
        #   'prj': multiply (\sqrt{d_model} ^ -1) to linear projection output
        #   'none': no multiplication

        assert scale_emb_or_prj in ['emb', 'prj', 'none']
        scale_emb = (scale_emb_or_prj == 'emb') if trg_emb_prj_weight_sharing else False
        self.scale_prj = (scale_emb_or_prj == 'prj') if trg_emb_prj_weight_sharing else False
        self.d_model = d_model

        self.encoder = Encoder(
            n_src_vocab=n_src_vocab, n_position=n_position,
            d_word_vec=d_word_vec, d_model=d_model, d_inner=d_inner,
            n_layers=n_layers, n_head=n_head, d_k=d_k, d_v=d_v,
            pad_idx=src_pad_idx, dropout=dropout, scale_emb=scale_emb)

        self.decoder = Decoder(
            n_trg_vocab=n_trg_vocab, n_position=n_position,
            d_word_vec=d_word_vec, d_model=d_model, d_inner=d_inner,
            n_layers=n_layers, n_head=n_head, d_k=d_k, d_v=d_v,
            pad_idx=trg_pad_idx, dropout=dropout, scale_emb=scale_emb)

        self.trg_word_prj = nn.Linear(d_model, n_trg_vocab, bias=False)

        for p in self.parameters():
            if p.dim() > 1:
                nn.init.xavier_uniform_(p) # 均匀分布初始化权重

        assert d_model == d_word_vec, \
        'To facilitate the residual connections, \
         the dimensions of all module outputs shall be the same.'

        if trg_emb_prj_weight_sharing:
            # Share the weight between target word embedding & last dense layer
            self.trg_word_prj.weight = self.decoder.trg_word_emb.weight

        if emb_src_trg_weight_sharing:
            self.encoder.src_word_emb.weight = self.decoder.trg_word_emb.weight


    def forward(self, src_seq, trg_seq):

        src_mask = get_pad_mask(src_seq, self.src_pad_idx)
        trg_mask = get_pad_mask(trg_seq, self.trg_pad_idx) & get_subsequent_mask(trg_seq)

        enc_output, *_ = self.encoder(src_seq, src_mask)
        dec_output, *_ = self.decoder(trg_seq, trg_mask, enc_output, src_mask)
        seq_logit = self.trg_word_prj(dec_output)
        if self.scale_prj:
            seq_logit *= self.d_model ** -0.5

        return seq_logit.view(-1, seq_logit.size(2))

# testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # 配置参数
    batch_size = 2
    seq_length = 5
    n_src_vocab = 12  # 源词汇表大小
    n_trg_vocab = 10  # 目标词汇表大小
    src_pad_idx = 0
    trg_pad_idx = 0
    d_word_vec = 8
    d_model = 8
    d_inner = 32
    n_layers = 2
    n_head = 2
    d_k = 4
    d_v = 4
    dropout = 0.1
    n_position = 50

    # 创建虚拟的 src 和 trg 数据
    src_seq = torch.tensor([[1, 2, 3, 4, 0], [4, 3, 2, 1, 0]], dtype=torch.long)  # shape: (batch_size, seq_length)
    trg_seq = torch.tensor([[1, 2, 3, 0, 0], [1, 3, 4, 5, 0]], dtype=torch.long)  # shape: (batch_size, seq_length)

    # 创建 Transformer 模型实例
    model = Transformer(
        n_src_vocab=n_src_vocab, n_trg_vocab=n_trg_vocab, 
        src_pad_idx=src_pad_idx, trg_pad_idx=trg_pad_idx,
        d_word_vec=d_word_vec, d_model=d_model, d_inner=d_inner,
        n_layers=n_layers, n_head=n_head, d_k=d_k, d_v=d_v, 
        dropout=dropout, n_position=n_position, 
        trg_emb_prj_weight_sharing=False, emb_src_trg_weight_sharing=False)

    # 前向传播
    with torch.no_grad():  # 不计算梯度
        src_mask = get_pad_mask(src_seq, src_pad_idx)
        logging.info("src_mask: %s", src_mask)
        
        trg_mask = get_pad_mask(trg_seq, trg_pad_idx) & get_subsequent_mask(trg_seq) # 逻辑与 掩码
        logging.info("trg_mask: %s", trg_mask)
        enc_output, *_ = model.encoder(src_seq, src_mask)
        logging.info("Encoder output shape: %s", enc_output.shape)
        
        dec_output, *_ = model.decoder(trg_seq, trg_mask, enc_output, src_mask)
        logging.info("Decoder output shape: %s", dec_output.shape)

        seq_logit = model.trg_word_prj(dec_output)
        logging.info("Final logits shape: %s", seq_logit.shape)
        print("Transformed output shape (viewed):", seq_logit.view(-1, seq_logit.size(2)).shape)