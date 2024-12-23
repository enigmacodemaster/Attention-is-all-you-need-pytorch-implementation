张量（Tensor）的维度操作是深度学习模型构建和数据预处理中的基本技能，大家估计平时写代码和看代码的时候经常会碰到这类张量维度变来边去的问题，有时候会搞混不同的操作，而且当需要对张量的维度进行调整、重排、拼接、拆分时，总是不知道应该采用什么操作来实现。

好的，那么本文就来解决这个问题~

本文将对PyTorch中常用的张量维度操作API进行全面总结，详细解释其用法及一般使用场景。

**超干预警！！！**

---

## 目录

1. [变形操作（Reshaping Operations）](#1-变形操作reshapeview)
   - [`reshape`](#reshape)
   - [`view`](#view)
   - [`flatten`](#flatten)
   - [`unflatten`](#unflatten)
2. [维度添加与删除](#2-维度添加与删除)
   - [`unsqueeze`](#unsqueeze)
   - [`squeeze`](#squeeze)
3. [维度重排与转置](#3-维度重排与转置)
   - [`permute`](#permute)
   - [`transpose`](#transpose)
   - [`swapaxes`](#swapaxes)
   - [`movedim`](#movedim)
4. [拼接与堆叠](#4-拼接与堆叠)
   - [`cat`](#cat)
   - [`stack`](#stack)
5. [拆分操作（Splitting Operations）](#5-拆分操作splitsplitchunk)
   - [`split`](#split)
   - [`chunk`](#chunk)
6. [扩展与重复](#6-扩展与重复expandrepeat)
   - [`expand`](#expand)
   - [`repeat`](#repeat)
7. [其他有用的维度操作](#7-其他有用的维度操作)
   - [`transpose`](#transpose-2)
   - [`unsqueeze`](#unsqueeze-2)
8. [实用示例与使用场景](#8-实用示例与使用场景)
9. [总结与建议](#9-总结与建议)

---

## 1. 变形操作（Reshaping Operations）

### `reshape`

**功能**：改变张量的形状（维度），不改变数据本身。新形状可以与原形状总元素数相同，也可以自动推断某个维度。

**语法**：

```python
tensor = tensor.reshape(new_shape)
```

**参数**：
- `new_shape`：整数或整数元组。可以使用`-1`自动推断维度。

**用法示例**：
```python
import torch

# 创建一个形状为 (2, 3) 的张量
x = torch.arange(6).reshape(2, 3)
print(x)
# 输出：
# tensor([[0, 1, 2],
#         [3, 4, 5]])

# 变形为 (3, 2)
y = x.reshape(3, 2)
print(y)
# 输出：
# tensor([[0, 1],
#         [2, 3],
#         [4, 5]])

# 使用 -1 自动推断维度
z = x.reshape(-1)
print(z)
# 输出：
# tensor([0, 1, 2, 3, 4, 5])
```

**一般使用场景**：
- 数据预处理，如将图像数据展平为向量输入全连接层。
- 在模型内部调整数据形状以匹配不同层的输入输出要求。

### `view`

**功能**：类似于`reshape`，改变张量的形状。区别在于`view`要求张量在内存中是连续的，否则需要先调用`contiguous()`。

**语法**：
```python
tensor = tensor.view(new_shape)
```

**参数**：
- `new_shape`：整数或整数元组。同`reshape`。

**用法示例**：
```python
import torch

# 创建一个不连续的张量
x = torch.randn(4, 4)
y = x.t()  # 转置后是非连续的

# 使用 view 前需要调用 contiguous()
try:
    y_view = y.view(16)
except RuntimeError as e:
    print(e)  # 抛出错误，因为 y 是非连续的

# 正确做法
y_contiguous = y.contiguous()
y_view = y_contiguous.view(16)
print(y_view.shape)
# 输出：
# torch.Size([16])
```

**一般使用场景**：
- 与`reshape`类似，但在需要确保张量连续性的特定情况下使用。
- 在模型中对特征图进行展平等操作。

### `flatten`

**功能**：将多维张量展平成一维或指定维度范围的张量。

**语法**：
```python
tensor = tensor.flatten(start_dim=0, end_dim=-1)
```

**参数**：
- `start_dim`：起始维度开始展平。
- `end_dim`：结束维度结束展平。默认为最后一维。

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4)
y = x.flatten()
print(y.shape)
# 输出：
# torch.Size([24])

y = x.flatten(1)
print(y.shape)
# 输出：
# torch.Size([2, 12])
```

**一般使用场景**：
- 在卷积神经网络中，将多维特征图展平特征向量，以输入全连接层。
- 减少维度以便于后续的处理或计算。

### `unflatten`

**功能**：将一维或低维张量还原为多维张量，根据指定的维度和形状进行还原。

**语法**：
```python
tensor = tensor.unflatten(dim, sizes)
```

**参数**：
- `dim`：要还原的维度。
- `sizes`：一个可迭代对象，指定还原后的每个维度的大小。

**用法示例**：
```python
import torch

x = torch.arange(12).reshape(3, 4)
print(x)
# 输出：
# tensor([[ 0,  1,  2,  3],
#         [ 4,  5,  6,  7],
#         [ 8,  9, 10, 11]])

y = x.unflatten(1, (2, 2))
print(y.shape)
# 输出：
# torch.Size([3, 2, 2])
print(y)
# 输出：
# tensor([[[ 0,  1],
#          [ 2,  3]],

#         [[ 4,  5],
#          [ 6,  7]],

#         [[ 8,  9],
#          [10, 11]]])
```

**一般使用场景**：
- 在还原展平后的特征向量到原始特征图形状时使用。
- 在处理特定形状转换时，如从全连接层还原到卷积层的输入形状。

---

## 2. 维度添加与删除

### `unsqueeze`

**功能**：在指定的位置添加一个大小为1的新维度，返回一个新的张量。

**语法**：
```python
tensor = tensor.unsqueeze(dim)
```

**参数**：
- `dim`：要添加新维度的位置。可以是负数，表示从最后一维开始倒数。

**用法示例**：
```python
import torch

x = torch.tensor([1, 2, 3])
print(x.shape)
# 输出：
# torch.Size([3])

y = x.unsqueeze(0)
print(y.shape)
# 输出：
# torch.Size([1, 3])

z = x.unsqueeze(-1)
print(z.shape)
# 输出：
# torch.Size([3, 1])
```

**一般使用场景**：
- 扩展张量维度以匹配操作需要的维度，如在批处理中添加批次维度。
- 改变张量的形状以便进行广播（broadcasting）等操作。

### `squeeze`

**功能**：移除指定位置或所有大小为1的维度，返回一个新的张量。

**语法**：
```python
tensor = tensor.squeeze(dim=None)
```

**参数**：
- `dim`：可选，指定要移除的维度，如果指定，则只有该维度大小为1时才被移除。如果未指定，移除所有大小为1的维度。

**用法示例**：
```python
import torch

x = torch.randn(1, 3, 1, 5)
print(x.shape)
# 输出：
# torch.Size([1, 3, 1, 5])

y = x.squeeze(0)
print(y.shape)
# 输出：
# torch.Size([3, 1, 5])

z = x.squeeze()
print(z.shape)
# 输出：
# torch.Size([3, 5])
```

**一般使用场景**：
- 简化张量形状，移除不必要的单维度。
- 改变张量的形状以便进行后续的操作或计算。

---

## 3. 维度重排与转置

### `permute`

**功能**：重新排列张量的维度，返回一个新的张量。

**语法**：
```python
tensor = tensor.permute(dims)
```

**参数**：
- `dims`：新的维度顺序，必须包含所有原始维度，并以元组或列表形式提供。

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4)
y = x.permute(2, 0, 1)
print(y.shape)
# 输出：
# torch.Size([4, 2, 3])
```

**一般使用场景**：
- 在不同框架或库之间迁移数据时，调整维度顺序。
- 将张量从通道最后（HWC）布局转换为通道第一（CHW）布局，常用于图像处理。
- 对多维张量进行维度重排以适应特定操作需求。

### `transpose`

**功能**：交换指定的两个维度，返回一个新的张量。

**语法**：
```python
tensor = tensor.transpose(dim0, dim1)
```

**参数**：
- `dim0`：第一个维度。
- `dim1`：第二个维度。

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4)
y = x.transpose(0, 1)
print(y.shape)
# 输出：
# torch.Size([3, 2, 4])
```

**一般使用场景**：
- 在需要交换特定维度顺序时使用，如从`(batch, channels, height, width)`转为`(batch, height, width, channels)`。
- 调整数据布局以满足操作层（如卷积层）对输入维度顺序的要求。

### `swapaxes`

**功能**：交换指定的两个维度，与`transpose`类似。PyTorch中通常用`transpose`完成。

**语法**：
```python
tensor = tensor.transpose(dim0, dim1)  # PyTorch推荐使用transpose
```

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4)
y = x.transpose(1, 2)
print(y.shape)
# 输出：
# torch.Size([2, 4, 3])
```

**一般使用场景**：
- 类似于`transpose`，用于交换张量的两个维度。

### `movedim`

**功能**：将一个或多个维度移动到新的位置，返回一个新的张量。

**语法**：
```python
tensor = tensor.movedim(source, destination)
```

**参数**：
- `source`：要移动的维度或维度列表。
- `destination`：新位置或位置列表。

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4, 5)
y = x.movedim(1, -1)
print(y.shape)
# 输出：
# torch.Size([2, 4, 5, 3])

# 移动多个维度
z = x.movedim([0, 2], [3, 1])
print(z.shape)
# 输出：
# torch.Size([3, 4, 2, 5])
```

**一般使用场景**：
- 当需要将多个维度重新排序或移动到不同的位置时使用，提供比`permute`更灵活的维度调整方式。
- 在复杂的张量处理过程中，需要调整多个维度以符合特定操作的输入要求。

---

## 4. 拼接与堆叠

### `cat`（Concatenate）

**功能**：在指定维度上拼接多个张量，返回一个新的张量。

**语法**：
```python
tensor = torch.cat(tensors, dim=0)
```

**参数**：
- `tensors`：待拼接的张量序列（列表或元组）。
- `dim`：拼接的维度。

**用法示例**：
```python
import torch

x = torch.randn(2, 3)
y = torch.randn(2, 3)
z = torch.cat([x, y], dim=0)
print(z.shape)
# 输出：
# torch.Size([4, 3])

z = torch.cat([x, y], dim=1)
print(z.shape)
# 输出：
# torch.Size([2, 6])
```

**一般使用场景**：
- 拼接批次数据。
- 在特征融合时，将不同来源的特征拼接到一起。
- 构建多层级特征图，如在U-Net架构中的跳跃连接。

### `stack`

**功能**：在新的维度上堆叠多个张量，使其增加一个新的维度。

**语法**：
```python
tensor = torch.stack(tensors, dim=0)
```

**参数**：
- `tensors`：待堆叠的张量序列（列表或元组），长度必须一致。
- `dim`：新增加的维度的位置。

**用法示例**：
```python
import torch

x = torch.tensor([1, 2, 3])
y = torch.tensor([4, 5, 6])
z = torch.stack([x, y], dim=0)
print(z)
# 输出：
# tensor([[1, 2, 3],
#         [4, 5, 6]])

z = torch.stack([x, y], dim=1)
print(z)
# 输出：
# tensor([[1, 4],
#         [2, 5],
#         [3, 6]])
```

**一般使用场景**：
- 创建批次维度，将多个样本堆叠到一起。
- 在序列模型中，将时间步的输出堆叠起来。
- 增加一个新的维度用于后续的处理，如广播操作。

### `torch.cat` 与 `torch.stack` 的区别

- **拼接 (`torch.cat`)**：沿现有维度将张量连接起来，不改变维度数。
  
- **堆叠 (`torch.stack`)**：在新的维度上堆叠张量，增加维度数。

**举例说明**：
```python
import torch

x = torch.randn(2, 3)
y = torch.randn(2, 3)

cat_0 = torch.cat([x, y], dim=0)  # 结果形状: [4, 3]
stack_0 = torch.stack([x, y], dim=0)  # 结果形状: [2, 2, 3]

print(cat_0.shape)    # torch.Size([4, 3])
print(stack_0.shape)  # torch.Size([2, 2, 3])
```

**总结**：
- 使用`cat`当需要在现有维度上扩展张量时。
- 使用`stack`当需要增加一个新的维度来堆叠张量时。

---

## 5. 拆分操作（Splitting Operations）

### `split`

**功能**：将张量沿指定维度分成多个块。每个块的大小可以相同或指定。

**语法**：
```python
chunks = torch.split(tensor, split_size_or_sections, dim=0)
```

**参数**：
- `split_size_or_sections`：每个块的大小，或一个包含每个块大小的列表/元组。
- `dim`：沿哪一个维度进行拆分。

**用法示例**：
```python
import torch

x = torch.arange(10)

# 均匀拆分，每个块大小为3，最后一个块会较小
chunks = torch.split(x, 3, dim=0)
for chunk in chunks:
    print(chunk)
# 输出：
# tensor([0, 1, 2])
# tensor([3, 4, 5])
# tensor([6, 7, 8])
# tensor([9])

# 按指定大小拆分
chunks = torch.split(x, [2, 3, 5], dim=0)
for chunk in chunks:
    print(chunk)
# 输出：
# tensor([0, 1])
# tensor([2, 3, 4])
# tensor([5, 6, 7, 8, 9])
```

**一般使用场景**：
- 在模型中将张量分成多个部分进行并行处理。
- 将数据分割成多个批次进行分布式训练。
- 在神经网络中按通道、批次等维度拆分特征图。

### `chunk`

**功能**：将张量沿指定维度按数量平均分割成多个块。如果无法平均分割，最后几个块可能会稍大。

**语法**：
```python
chunks = torch.chunk(tensor, chunks, dim=0)
```

**参数**：
- `chunks`：要分割成多少个块。
- `dim`：沿哪一个维度进行分割。

**用法示例**：
```python
import torch

x = torch.arange(10)

# 将张量分成3块，尽可能均匀
chunks = torch.chunk(x, 3, dim=0)
for chunk in chunks:
    print(chunk)
# 输出：
# tensor([0, 1, 2, 3])
# tensor([4, 5, 6])
# tensor([7, 8, 9])
```

**一般使用场景**：
- 与`split`类似，用于在指定维度上按数量分割张量。
- 在GPU并行计算中，将数据划分到不同设备或处理单元。

**`split` vs `chunk` 的区别**：
- `split` 更灵活，支持按大小分割。
- `chunk` 只能按块数分割，尽量平均。

---

## 6. 扩展与重复

### `expand`

**功能**：扩展张量的尺寸，不复制数据，返回一个可广播的张量。

**语法**：
```python
tensor = tensor.expand(*sizes)
```

**参数**：
- `sizes`：新的尺寸，可以使用`-1`保持原尺寸，或者用整数指定扩展的维度大小。

**用法示例**：
```python
import torch

x = torch.tensor([1, 2, 3])
y = x.unsqueeze(0)  # shape [1, 3]
z = y.expand(4, 3)
print(z)
# 输出：
# tensor([[1, 2, 3],
#         [1, 2, 3],
#         [1, 2, 3],
#         [1, 2, 3]])
```

**注意**：
- 使用`expand`不会复制数据，返回的张量是原张量的“视图”。因此，不可修改扩展后的张量。

**一般使用场景**：
- 与更高维度的张量进行广播操作时使用。
- 在模型中需要重复某些向量或特征以匹配形状需求。

### `repeat`

**功能**：沿指定的维度重复张量的内容，返回一个新的张量。

**语法**：
```python
tensor = tensor.repeat(*repeats)
```

**参数**：
- `repeats`：每个维度上重复的次数，必须与张量的维度数匹配。

**用法示例**：
```python
import torch

x = torch.tensor([1, 2, 3])
y = x.repeat(2)  # [1, 2, 3, 1, 2, 3]
z = x.repeat(2, 3)  # [[1, 2, 3, 1, 2, 3, 1, 2, 3],
                     #  [1, 2, 3, 1, 2, 3, 1, 2, 3]]
print(y)
print(z)
```

**一般使用场景**：
- 在需要复制数据以符合特定形状要求时使用，如在批处理中的特征复制。
- 在实现某些特定的数学运算或模型结构时需要重复数据。

**区别 `expand` 与 `repeat`**：
- `expand` 不会复制数据，仅改变张量的视图，效率更高，但有广播限制。
- `repeat` 会实际复制数据，生成新的张量，适用于需要完全复制数据的场景。

---

## 7. 其他有用的维度操作

### `transpose`（再次介绍）

**功能**：在指定的两个维度之间交换位置。

**语法**：
```python
tensor = tensor.transpose(dim0, dim1)
```

**参数**：
- `dim0`：要交换的第一个维度。
- `dim1`：要交换的第二个维度。

**用法示例**：
```python
import torch

x = torch.randn(2, 3, 4)
y = x.transpose(1, 2)
print(y.shape)
# 输出：
# torch.Size([2, 4, 3])
```

**一般使用场景**：
- 如前所述，用于调整数据的维度顺序以匹配不同层或操作的需求。

### `unsqueeze`（再次介绍）

**功能**：在指定的位置添加一个大小为1的新维度。

**语法**：
```python
tensor = tensor.unsqueeze(dim)
```

**参数**：
- `dim`：要添加新维度的位置。

**用法示例**：
```python
import torch

x = torch.tensor([1, 2, 3])
y = x.unsqueeze(0)  # shape [1, 3]
z = x.unsqueeze(1)  # shape [3, 1]
print(y.shape, z.shape)
# 输出：
# torch.Size([1, 3]) torch.Size([3, 1])
```

**一般使用场景**：
- 与`expand`或`repeat`结合使用，以调整数据形状符合广播或其他操作需求。

---

## 8. 实用示例与使用场景

为更好地理解上述API的使用，以下通过几个常见的深度学习任务示例展示张量维度操作的应用。

### 示例1：图像数据预处理

**任务**：将单张RGB图像转换为模型输入的批次张量格式。

**步骤**：
1. 初始形状为 `[H, W, 3]`（高度、宽度、通道）。
2. 将通道维度移到第一位，变为 `[3, H, W]`。
3. 添加批次维度，变为 `[1, 3, H, W]`。

**代码示例**：
```python
import torch

# 假设有一张 RGB 图像，形状为 (H, W, 3)
image = torch.randn(224, 224, 3)

# 移动通道维度到第一维
image = image.permute(2, 0, 1)  # shape [3, 224, 224]

# 添加批次维度
image = image.unsqueeze(0)  # shape [1, 3, 224, 224]

print(image.shape)
# 输出：
# torch.Size([1, 3, 224, 224])
```

### 示例2：特征图调整

**任务**：将卷积层的输出特征图从 `[batch, channels, height, width]` 转换为 `[batch, height, width, channels]`，以便于某些操作如注意力机制的应用。

**步骤**：
1. 使用`permute`调整维度顺序。

**代码示例**：
```python
import torch

# 假设有一个卷积层输出的特征图，形状为 [batch, channels, height, width]
features = torch.randn(16, 64, 28, 28)

# 调整为 [batch, height, width, channels]
features = features.permute(0, 2, 3, 1)

print(features.shape)
# 输出：
# torch.Size([16, 28, 28, 64])
```

### 示例3：批次数据拼接

**任务**：将多个特征向量沿批次维度拼接成一个大的批次。

**步骤**：
1. 使用`cat`或`stack`在批次维度拼接。

**代码示例**：
```python
import torch

# 假设有两个批次的特征向量，形状为 [batch_size, features]
batch1 = torch.randn(8, 128)
batch2 = torch.randn(8, 128)

# 拼接为一个批次，增加批次大小
combined = torch.cat([batch1, batch2], dim=0)  # shape [16, 128]

print(combined.shape)
# 输出：
# torch.Size([16, 128])
```

### 示例4：特征复制与广播

**任务**：为每个样本重复一个向量，以便与特征图相加。

**步骤**：
1. 使用`unsqueeze`和`repeat`或`expand`调整维度。
2. 进行广播操作。

**代码示例**：
```python
import torch

# 假设有一种全局特征向量，形状为 [features]
global_feat = torch.randn(128)

# 有一个特征图，形状为 [batch, channels, height, width]
feature_map = torch.randn(16, 128, 32, 32)

# 调整全局特征的形状为 [batch, channels, height, width]
global_feat = global_feat.unsqueeze(0).unsqueeze(-1).unsqueeze(-1)  # shape [1, 128, 1, 1]
global_feat = global_feat.expand(-1, -1, 32, 32)  # shape [1, 128, 32, 32]
global_feat = global_feat.repeat(16, 1, 1, 1)  # shape [16, 128, 32, 32]

# 也可以使用 `broadcast_to` 进行广播
# PyTorch 1.10+ 支持 torch.broadcast_to
# global_feat = global_feat.broadcast_to(16, 128, 32, 32)

# 现在可以与特征图相加
combined_feat = feature_map + global_feat

print(combined_feat.shape)
# 输出：
# torch.Size([16, 128, 32, 32])
```

---

## 9. 总结与建议

PyTorch提供了丰富且灵活的张量维度操作API，能满足各种深度学习模型构建和数据预处理的需求。以下是一些关键点和建议：

1. **理解维度的重要性**：
   - 深度学习模型中，数据形状（如批次维度、通道维度等）的正确性至关重要。理解各API如何改变张量形状，有助于避免形状不匹配的问题。

2. **选择合适的API**：
   - 根据任务需求选择合适的维度操作API。例如，`reshape`适合调整形状，`permute`用于维度重排，`cat`和`stack`用于拼接与堆叠。

3. **保持代码可读性**：
   - 维度操作往往涉及多个步骤，确保代码中对维度变化的操作逻辑清晰，添加注释以解释维度如何被调整。

4. **性能考虑**：
   - 尽量使用不复制数据的操作（如`permute`, `view`, `expand`），减少内存开销和计算量，提升性能。

5. **调试工具**：
   - 利用`print(tensor.shape)`或`tensor.size()`辅助调试，确保每一步维度操作正确。

6. **结合实际案例学习**：
   - 通过实际的深度学习项目，如图像分类、目标检测、语义分割等任务，练习并巩固张量维度操作的应用。

7. **深入理解广播机制**：
   - 理解PyTorch的广播规则，合理利用`expand`和`repeat`等方法，实现高效的批量运算。

8. **学习官方文档与社区资源**：
   - 参考[PyTorch官方文档](https://pytorch.org/docs/stable/tensors.html)和社区示例，获取更多维度操作的应用示例和最佳实践。

通过系统性地掌握和实践这些张量维度操作的API，您将能够更加高效地构建和优化深度学习模型，解决实际问题中的形状匹配和数据处理挑战。

---

## 附录：常用维度操作API快速参考表

| **API**      | **功能**                                | **典型用法**                             |
|--------------|----------------------------------------|------------------------------------------|
| `reshape`    | 改变张量形状                            | `x.reshape(4, 3)`                        |
| `view`       | 改变张量形状（要求连续）                | `x.view(4, 3)`                           |
| `flatten`    | 展平张量为一维或指定维度范围            | `x.flatten(1)`                           |
| `unflatten`  | 将低维张量还原为多维                    | `x.unflatten(1, (2, 2))`                 |
| `unsqueeze`  | 在指定维度添加大小为1的新维度            | `x.unsqueeze(0)`                         |
| `squeeze`    | 移除大小为1的指定或所有维度              | `x.squeeze(1)` 或 `x.squeeze()`           |
| `permute`    | 重排张量的维度                          | `x.permute(2, 0, 1)`                      |
| `transpose`  | 交换指定的两个维度                        | `x.transpose(0, 1)`                       |
| `movedim`    | 将一个或多个维度移动到新的位置            | `x.movedim(0, -1)`                        |
| `cat`        | 在指定维度拼接多个张量                    | `torch.cat([x, y], dim=0)`                |
| `stack`      | 在新的维度堆叠多个张量                    | `torch.stack([x, y], dim=1)`              |
| `split`      | 按指定大小沿维度拆分张量                  | `torch.split(x, 3, dim=0)`                |
| `chunk`      | 按指定块数沿维度拆分张量                  | `torch.chunk(x, 2, dim=0)`                |
| `expand`     | 扩展张量，不复制数据                      | `x.expand(4, 3)`                           |
| `repeat`     | 重复张量数据，复制生成新张量               | `x.repeat(2, 3)`                           |

---

通过以上内容，您可以全面了解并掌握PyTorch中关于张量维度操作的各种API及其应用场景，从而在实际开发和模型构建中更加得心应手。
