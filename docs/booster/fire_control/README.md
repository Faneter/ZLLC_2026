# 火控策略

火控策略位于文件[`crt_booster.cpp`](../../../User/Chariot/crt_booster.cpp)中的`Class_Booster::Output`函数中的`switch`分支中的`Booster_Control_Type_REPEATED`分支项中。

## 策略来源

火控策略涉及到的算法来自同济大学的[这篇文章](https://bbs.robomaster.com/article/630409)中。

然后更具体的控制策略又参考了辽宁科技大学的[仓库代码](https://gitee.com/cod_-control/rmcod2026_-sentry#%E7%83%AD%E9%87%8F%E6%8E%A7%E5%88%B6)

## 策略实现

### 定义用到的量

根据文章内容，我们先拿出其中的三个量`a`、`m`、`d`

```cpp
// 机器人冷却(单位：点/s)
float a = static_cast<float>(Referee->Get_Booster_17mm_1_Heat_CD());
// 枪口还能利用的热量上限(单位：点)
float m = static_cast<float>(Referee->Get_Booster_17mm_1_Heat_Max() - FSM_Heat_Detect.Heat);
// 每射击一发消耗热量(单位：点)
constexpr float d = 10.0f;
```

然后再定义依据这些量计算出的具体控制参数。

```cpp
// 射速(单位：发/s)
static float shoot_speed = 0.0f;
// 本周期射击的总持续时间(单位：ms)
static uint16_t ShootTime = 0;
// 本周期射击的实际时间(单位：ms)
static uint16_t shoot_time      = 0;
static uint16_t last_shoot_time = 0;
```

最后是实际的拨弹盘转速及其与发射速度的转换关系。

```cpp
// 最终得出的拨弹盘转速
float target_omega = 0.0f;
// 射速与拨弹盘转速的转换关系
constexpr float rad_per_bullet = 2.0f * PI / 9.0f; // 假设一圈9发
```

### 具体控制算法

具体的控制算法部分，详见源文件的代码中的注释。
