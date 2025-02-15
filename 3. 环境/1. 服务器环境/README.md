<!--
 * @Author: Ken Kaneki
 * @Date: 2021-06-03 23:39:10
 * @LastEditTime: 2021-06-04 00:10:47
 * @Description: README
 * @FilePath: \undefinedd:\Learn\毕设资料\毕业设计-李圭印\3. 环境\1. 服务器环境\README.md
-->
# 操作系统
    Ubuntu18.04
# 开发平台
    [Baidu AI Studio](https://aistudio.baidu.com/aistudio/usercenter)
# 版本号
|  组件  | 版本   |
| :-----------: | :------------: |
| paddlepaddle  | 2.0.2          |
| paddleDection | 2.0.0          |
| paddleSlim    | 2.0.0          |
| paddleLite    | 2.8            |
| OpenCV        | 3.2.0          |
# paddlepaddle安装
[官方安装网址](https://www.paddlepaddle.org.cn/install/quick?docurl=/documentation/docs/zh/install/pip/windows-pip.html)

<p align="center">
<img align="center" src="doc/imgs/logo.png", width=1600>
<p>

--------------------------------------------------------------------------------

[English](./README.md) | 简体中文

[![Build Status](https://travis-ci.org/PaddlePaddle/Paddle.svg?branch=develop)](https://travis-ci.org/PaddlePaddle/Paddle)
[![Documentation Status](https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat)](https://paddlepaddle.org.cn/documentation/docs/en/guides/index_en.html)
[![Documentation Status](https://img.shields.io/badge/中文文档-最新-brightgreen.svg)](https://paddlepaddle.org.cn/documentation/docs/zh/guides/index_cn.html)
[![Release](https://img.shields.io/github/release/PaddlePaddle/Paddle.svg)](https://github.com/PaddlePaddle/Paddle/releases)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](LICENSE)

欢迎来到 PaddlePaddle GitHub

飞桨(PaddlePaddle)以百度多年的深度学习技术研究和业务应用为基础，是中国首个自主研发、功能完备、 开源开放的产业级深度学习平台，集深度学习核心训练和推理框架、基础模型库、端到端开发套件和丰富的工具组件于一体。目前，飞桨累计开发者265万，服务企业10万家，基于飞桨开源深度学习平台产生了34万个模型。飞桨助力开发者快速实现AI想法，快速上线AI业务。帮助越来越多的行业完成AI赋能，实现产业智能化升级。

## 安装

### PaddlePaddle最新版本: [v2.0](https://github.com/PaddlePaddle/Paddle/tree/release/2.0)

跟进PaddlePaddle最新特性请参考我们的[版本说明](https://github.com/PaddlePaddle/Paddle/releases)

### 安装最新稳定版本:
```
# CPU
pip install paddlepaddle
# GPU
pip install paddlepaddle-gpu
```
更多安装信息详见官网 [安装说明](https://www.paddlepaddle.org.cn/install/quick)

PaddlePaddle用户可领取**免费Tesla V100在线算力资源**，训练模型更高效。**每日登陆即送10小时**，[前往使用免费算力](https://aistudio.baidu.com/aistudio/index)。

## 四大领先技术

- **开发便捷的产业级深度学习框架**

    飞桨深度学习框架采用基于编程逻辑的组网范式，对于普通开发者而言更容易上手，符合他们的开发习惯。同时支持声明式和命令式编程，兼具开发的灵活性和高性能。网络结构自动设计，模型效果超越人类专家。


- **支持超大规模深度学习模型的训练**

    飞桨突破了超大规模深度学习模型训练技术，实现了支持千亿特征、万亿参数、数百节点的开源大规模训练平台，攻克了超大规模深度学习模型的在线学习难题，实现了万亿规模参数模型的实时更新。
    [查看详情](https://github.com/PaddlePaddle/Fleet)


- **多端多平台部署的高性能推理引擎**

    飞桨不仅兼容其他开源框架训练的模型，还可以轻松地部署到不同架构的平台设备上。同时，飞桨的推理速度也是全面领先的。尤其经过了跟华为麒麟NPU的软硬一体优化，使得飞桨在NPU上的推理速度进一步突破。
    [查看详情](https://github.com/PaddlePaddle/Paddle-Lite)


- **面向产业应用，开源开放覆盖多领域的工业级模型库。**

    飞桨官方支持100多个经过产业实践长期打磨的主流模型，其中包括在国际竞赛中夺得冠军的模型；同时开源开放200多个预训练模型，助力快速的产业应用。
    [查看详情](https://github.com/PaddlePaddle/models)


## 文档

我们提供 [英文](https://www.paddlepaddle.org.cn/documentation/docs/en/guides/index_en.html) 和
[中文](https://www.paddlepaddle.org.cn/documentation/docs/zh/guides/index_cn.html) 文档

- [使用指南](https://www.paddlepaddle.org.cn/documentation/docs/zh/guides/index_cn.html)

   或许您想从深度学习基础开始学习飞桨

- [应用实践](https://www.paddlepaddle.org.cn/documentation/docs/zh/tutorial/index_cn.html)


- [API Reference](https://www.paddlepaddle.org.cn/documentation/docs/zh/api/index_cn.html)

   新的API支持代码更少更简洁的程序


- [贡献方式](https://www.paddlepaddle.org.cn/documentation/docs/zh/guides/08_contribution/index_cn.html)

   欢迎您的贡献!

## 交流与反馈

- 欢迎您通过[Github Issues](https://github.com/PaddlePaddle/Paddle/issues)来提交问题、报告与建议
- QQ群: 793866180 (PaddlePaddle)
- [论坛](https://ai.baidu.com/forum/topic/list/168): 欢迎大家在PaddlePaddle论坛分享在使用PaddlePaddle中遇到的问题和经验, 营造良好的论坛氛围

## 版权和许可证
PaddlePaddle由[Apache-2.0 license](LICENSE)提供
# paddleDection安装
[官方安装网址](https://github.com/PaddlePaddle/PaddleDetection/blob/release/2.1/docs/tutorials/INSTALL_cn.md)
[English](INSTALL.md) | 简体中文


# 安装文档



## 环境要求

- PaddlePaddle 2.1
- OS 64位操作系统
- Python 3(3.5.1+/3.6/3.7/3.8/3.9)，64位版本
- pip/pip3(9.0.1+)，64位版本
- CUDA >= 10.1
- cuDNN >= 7.6

PaddleDetection 依赖 PaddlePaddle 版本关系：

|  PaddleDetection版本  | PaddlePaddle版本  |    备注    |
| :------------------: | :---------------: | :-------: |
|    release/2.1       |       >= 2.1.0    |     默认使用动态图模式    |
|    release/2.0       |       >= 2.0.1    |     默认使用动态图模式    |
|    release/2.0-rc    |       >= 2.0.1    |     --    |
|    release/0.5       |       >= 1.8.4    |  大部分模型>=1.8.4即可运行，Cascade R-CNN系列模型与SOLOv2依赖2.0.0.rc版本 |
|    release/0.4       |       >= 1.8.4    |  PP-YOLO依赖1.8.4 |
|    release/0.3       |        >=1.7      |     --    |

## 安装说明

### 1. 安装PaddlePaddle

```
# CUDA10.1
python -m pip install paddlepaddle-gpu==2.1.0.post101 -f https://paddlepaddle.org.cn/whl/mkl/stable.html

# CPU
python -m pip install paddlepaddle -i https://mirror.baidu.com/pypi/simple
```
- 更多CUDA版本或环境快速安装，请参考[PaddlePaddle快速安装文档](https://www.paddlepaddle.org.cn/install/quick)
- 更多安装方式例如conda或源码编译安装方法，请参考[PaddlePaddle安装文档](https://www.paddlepaddle.org.cn/documentation/docs/zh/install/index_cn.html)

请确保您的PaddlePaddle安装成功并且版本不低于需求版本。使用以下命令进行验证。

```
# 在您的Python解释器中确认PaddlePaddle安装成功
>>> import paddle
>>> paddle.utils.run_check()

# 确认PaddlePaddle版本
python -c "import paddle; print(paddle.__version__)"
```
**注意**
1. 如果您希望在多卡环境下使用PaddleDetection，请首先安装NCCL

### 2. 安装PaddleDetection

可通过如下两种方式安装PaddleDetection

#### 2.1 通过pip安装

**注意：** pip安装方式只支持Python3

```
# pip安装paddledet
pip install paddledet==2.1.0 -i https://mirror.baidu.com/pypi/simple

# 下载使用源码中的配置文件和代码示例
git clone https://github.com/PaddlePaddle/PaddleDetection.git
cd PaddleDetection
```

#### 2.2 源码编译安装

```
# 克隆PaddleDetection仓库
cd <path/to/clone/PaddleDetection>
git clone https://github.com/PaddlePaddle/PaddleDetection.git

# 编译安装paddledet
cd PaddleDetection
python setup.py install

# 安装其他依赖
pip install -r requirements.txt

```

**注意**

1. 若您使用的是Windows系统，由于原版cocoapi不支持Windows，`pycocotools`依赖可能安装失败，可采用第三方实现版本，该版本仅支持Python3

    ```pip install git+https://github.com/philferriere/cocoapi.git#subdirectory=PythonAPI```

2. 若您使用的是Python <= 3.6的版本，安装`pycocotools`可能会报错`distutils.errors.DistutilsError: Could not find suitable distribution for Requirement.parse('cython>=0.27.3')`, 您可通过先安装`cython`如`pip install cython`解决该问题


安装后确认测试通过：

```
python ppdet/modeling/tests/test_architectures.py
```

测试通过后会提示如下信息：

```
.....
----------------------------------------------------------------------
Ran 5 tests in 4.280s
OK
```

## 快速体验

**恭喜！** 您已经成功安装了PaddleDetection，接下来快速体验目标检测效果

```
# 在GPU上预测一张图片
export CUDA_VISIBLE_DEVICES=0
python tools/infer.py -c configs/ppyolo/ppyolo_r50vd_dcn_1x_coco.yml -o use_gpu=true weights=https://paddledet.bj.bcebos.com/models/ppyolo_r50vd_dcn_1x_coco.pdparams --infer_img=demo/000000014439.jpg
```

会在`output`文件夹下生成一个画有预测结果的同名图像。

结果如下图：

![](../images/000000014439.jpg)
