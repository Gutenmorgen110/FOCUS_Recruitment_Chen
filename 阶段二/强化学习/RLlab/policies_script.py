import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as D
import numpy as np
import matplotlib.pyplot as plt

torch.manual_seed(0)
# 一个最小 MLP：输入 4 维 obs，输出 2 维 logits（对应 CartPole 两个动作）
logits_net = nn.Sequential(
    nn.Linear(4, 32), nn.Tanh(),
    nn.Linear(32, 2)
)

# obs = torch.tensor([[0.03, -0.01, 0.04, 0.02]])   # 一个观测
# logits = logits_net(obs)
# # print("网络原始输出 logits =", logits)
# # print("形状 =", logits.shape)

# # print("*" * 80)
# dist = D.Categorical(logits=logits)
# print("分布对象：", dist)
# print("各动作概率：", dist.probs)

# # 采样 5 次，看随机性
# for i in range(5):
#     a = dist.sample()
#     print(f"第 {i+1} 次采样动作 = {a.item()}")

# actions = torch.tensor([0, 1, 0, 1, 1])
# log_probs = dist.log_prob(actions)
# advantages = torch.tensor([1.0,2.0,-1.0,0.5,-0.5])
# terms = log_probs*advantages
# loss = -terms.mean()
# logits_net.zero_grad()
# loss.backward()

# for name, p in logits_net.named_parameters():
#     print(f"{name}: grad.shape = {p.grad.shape}, "
#           f"grad.norm = {p.grad.norm().item():.4f}")
# print("log π(a|s) =", log_probs)
# print("advantage  =", advantages)
# print("乘积       =", terms)
# print("loss       =", loss.item())
# print(loss)
# print("动作序列 =", actions)
# print("log π(a|s) =", log_probs)
# print("对应的 π(a|s) =", log_probs.exp())



# ########################### 综合起来的一次总体实验
# optimizer = torch.optim.Adam(logits_net.parameters(), lr=0.1)
# # 记载一个学习率,第一个参数是要优化的参数列表

# obs = torch.tensor([[0.03, -0.01, 0.04, 0.02]])
# actions = torch.tensor([1])        # 希望网络学会做动作 1
# advantages = torch.tensor([5.0])   # 动作 1 很好

# for step in range(20):
#     logits = logits_net(obs)
#     dist = D.Categorical(logits=logits)
#     log_prob = dist.log_prob(actions)
#     loss = -(log_prob * advantages).mean()

#     optimizer.zero_grad()#清空旧的梯度
#     loss.backward()# 将梯度结果存入到p.norm中
#     optimizer.step()# 用新的梯度更新参数

#     if step % 5 == 0:
#         p = F.softmax(logits, dim=-1).detach()
#         print(f"step {step:2d}  π(a=0)={p[0,0]:.3f}  π(a=1)={p[0,1]:.3f}  loss={loss.item():.3f}")
    
# mean_net = nn.Sequential(
#     nn.Linear(4, 32), nn.Tanh(),
#     nn.Linear(32, 2)
# )
# logstd = nn.Parameter(torch.zeros(2))   # 独立参数，不依赖 obs

# obs = torch.tensor([[0.03, -0.01, 0.04, 0.02]])
# mean = mean_net(obs)
# std = torch.exp(logstd)

# # print("连续 mean.shape =", mean.shape)        # [3, 2]
# # print("连续 std.shape  =", std.shape)         # [2] —— 注意！不依赖 batch
# # print("mean =", mean)
# # print("std  =", std)
# # print("logst =",logstd)

# # dist = D.Normal(mean, std)
# # print("采样 3 次：", [dist.sample().numpy() for _ in range(3)])
# dist_c = D.Normal(mean, std)
# # samples_c = [dist_c.sample() for _ in range(5)]
# # for s in samples_c:
# #     print("连续采样：", s.numpy())   # 每维是实数，如 [0.12, -0.87]

# act_c = torch.randn(3, 2)                  # 每个样本一个 2 维动作
# lp_c = dist_c.log_prob(act_c)
# print("连续 log_prob.shape =", lp_c.shape)  # [3, 2] —— 注意！
# print("act_c",act_c)
# print(lp_c)



import torch, torch.nn as nn, torch.distributions as D

torch.manual_seed(0)
mean_net = nn.Sequential(nn.Linear(1, 32), nn.Tanh(), nn.Linear(32, 1))
logstd = nn.Parameter(torch.zeros(1))
opt = torch.optim.Adam(list(mean_net.parameters()) + [logstd], lr=5e-3)
# print(list(mean_net.parameters()))
# print([logstd])

obs = torch.zeros(1, 1)
target = 2.0
B = 64   # 关键 1：批量采样

for step in range(500):
    mean = mean_net(obs).expand(B, 1)          # [B,1]广播机制，将结果广播到这个B*1的矩阵中
    std  = torch.exp(logstd).expand(B, 1)      # [B,1]
    dist = D.Normal(mean, std)
    a = dist.sample()                          # [B,1]

    reward = -(a - target).abs().sum(dim=-1)   # [B]
    adv = reward - reward.mean()               # 关键 2：减 baseline

    log_prob = dist.log_prob(a).sum(dim=-1)    # [B]
    loss = -(log_prob * adv).mean()

    opt.zero_grad(); loss.backward(); opt.step()

    if step % 50 == 0:
        m = mean_net(obs).item()
        s = torch.exp(logstd).item()
        print(f"step {step:3d}  mean={m:+.3f}  std={s:.3f}")