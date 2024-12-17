```python
class LabelSmoothingLoss(nn.Module):
    def __init__(self, label_smoothing, vocab_size, ignore_index=-100):
        super(LabelSmoothingLoss, self).__init__()
        self.criterion = nn.KLDivLoss(reduction='sum')
        self.label_smoothing = label_smoothing
        self.vocab_size = vocab_size
        self.ignore_index = ignore_index

    def forward(self, logits, target):
        # logits: [batch_size * tgt_seq_len, vocab_size]
        # target: [batch_size * tgt_seq_len]
        
        # 计算 log_probs
        log_probs = F.log_softmax(logits, dim=-1)
        
        # 创建真实分布
        true_dist = torch.zeros_like(log_probs)
        true_dist.fill_(self.label_smoothing / (self.vocab_size - 1))
        ignore = target == self.ignore_index
        target = target.unsqueeze(1)
        true_dist.scatter_(1, target, 1.0 - self.label_smoothing)
        true_dist[ignore] = 0
        
        loss = self.criterion(log_probs, true_dist)
        return loss / (~ignore).sum()


criterion = LabelSmoothingLoss(label_smoothing=0.1, vocab_size=10000, ignore_index=0)
criterion(logits, targets)

```
