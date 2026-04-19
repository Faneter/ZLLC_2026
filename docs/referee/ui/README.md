# UI

## 入口

UI绘制的入口函数位于文件[`dvc_GraphicsSendTask.cpp`](../../../User/Device/dvc_GraphicsSendTask.cpp)中的`GraphicSendtask`函数。

此函数先进行UI的初始化绘制，然后使用状态机管理并将更新后的数据同步到UI上。

## 具体模块的绘制

模块绘制函数一般分为两部分，一部分是对用到的图形的创建，此部分只执行一次；另一部分是对会变动的图形的更新。

### 枪口热量

枪口热量绘制的主体思路是计算得出当前热量与热量上限的比值，然后绘制对应角度的圆弧。

对应的函数为`Booster_Heat_Draw`。
