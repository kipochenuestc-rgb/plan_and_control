# plan_and_control · AutoDriving lab

双击 `start.bat`，浏览器打开 http://127.0.0.1:8000 。当前电脑已经配置好 `.venv`；换电脑时安装 Python 3.11+，运行 `setup.bat`。

Vue 3 使用本地 vendored 浏览器构建，不需要 Node、npm 或 CDN；FastAPI 同时提供静态页面和 API。无需 ROS。

界面包含六个页签：

- MPC 路径跟踪：独立纵向 PI、横向 MPC，可显示路径局部 Frenet 坐标。
- 阿克曼运动学验证：WASD 驾驶，可调轴距、轮距、加速度与转角限制。
- 路径规划：二维栅格 A* / Dijkstra，支持起终点与障碍编辑。
- 图搜索：加权路网搜索，支持单步查看候选集合、代价更新与路径回溯。
- 行为规划：巡航、跟车、绕障、停车等待和红绿灯场景。
- 双路口综合仿真：路线搜索、封路绕行、信号灯与直行 / 转弯行为联合演示。

规划和行为页使用教学级简化运动执行，尚未接入 MPC；不代表完整 Autoware 实现。虚拟环境、日志、缓存和调试截图不纳入 Git。

## 结构

- `backend/simulation.py`：参考路径、MPC、独立速度 PI、车辆模型。
- `backend/app.py`：`/api/state`、`/api/reset`、`/api/step`，以及静态文件服务。
- `frontend/`：Vue 页面、Canvas 绘图。`vendor/vue.global.prod.js` 为 Vue 3.5.13 官方 MIT 浏览器构建。
- `test_simulation.py`：闭环收敛、约束、停车以及 API 验证。

## 控制原理

这是参考 Autoware 常规横向 MPC 思路的教学实现，并非源码移植。

状态 z=[e_y,e_psi,delta_actual]，输入 u=delta_command；速度为当前测量值，在预测窗口内保持不变，不参与横向优化。

误差模型：e_y_dot ≈ v*e_psi；e_psi_dot ≈ v/L*tan(delta)-v*kappa_ref；delta_dot=(u-delta)/tau。围绕 delta_ref=atan(L*kappa_ref) 线性化 tan，Euler 离散化，保留曲率仿射项。模型采用小误差近似。

将未来状态压缩为 Z=G*U+b；最小化横向/航向误差、偏离参考转角的代价和转角命令变化的代价，附加末端误差权重。OSQP 每周期求解有约束 QP，只执行第一步。

参数：轴距2.8m、转向一阶时间常数0.3s、控制周期0.1s、预测25步、命令转角±30°、命令变化率±45°/s。车辆使用非线性后轴自行车模型，0.01秒子步积分。独立 PI 控制速度，加速度限制[-3,2]m/s²，路径末端使用剩余距离减速。

QP失败时保持上一转角并制动，前端显示状态。场景为无障碍平面正向S形路线，无侧滑/轮胎力模型、无倒车、无障碍约束。不用于实车。服务仅监听本机，单进程共享一个仿真实例；多个浏览器会操作同一车辆。

阿克曼页以**后轴中心**作为状态位置，使用 x_dot=v*cos(psi)、y_dot=v*sin(psi)、psi_dot=v*tan(delta)/L。等效前轮转角给出后轴转弯半径 R=L/tan(delta)，再用轴距 L、轮距 T 计算内外前轮角，因此绘制的左右前轮会对向瞬时转动中心。这个页签不调用后端控制器，目的是单独验证纯运动学关系。

运行测试：`.venv\Scripts\python.exe -m unittest test_simulation -v`（API测试另需 httpx，已在当前环境安装）。

官方原理参考：https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/model_predictive_control_algorithm/
