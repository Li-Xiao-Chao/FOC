# 双闭环FOC

参考自《现代永磁同步电机控制原理及MATLAB仿真》袁雷编著

<img src=".\README.assets\image-20221206171437211.png" alt="image-20221206171437211" style="zoom: 50%;" />

<img src=".\README.assets\image-20221206214923151.png" alt="image-20221206214923151" style="zoom: 67%;" />

## 1.Clark变换

### 自然坐标系->α-β坐标系(Clark变换)

$$
\begin{bmatrix}
f_α&f_β&f_0
\end{bmatrix}^T
=T_{3S/2S}
\begin{bmatrix}
f_A&f_B&f_C
\end{bmatrix}^T
$$

f代表电机的电压、电流或磁链等变量；T<sub>3s/2s</sub>为坐标变换矩阵


$$
T_{3S/2S}=k\begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\\\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \\\\ \frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} &\frac{\sqrt{2}}{2} \end{bmatrix}\\\\
幅值不变k=\frac{2}{3}\\\\
功率不变k=\sqrt{\frac{2}{3}}\\\\
三相对称系统，静止坐标系中f_0可忽略
$$



<img src=".\README.assets\image-20221205095333937.png" alt="image-20221205095333937" style="zoom: 80%;" />



```c
void Clark_cal(void)
{ 
  Clark.Alpha = (Clark.A - (Clark.B + Clark.C) * 0.5) * 2.0 / 3.0;
  Clark.Beta = (Clark.B - Clark.C) * 0.8660254037844386 * 2.0 / 3.0;
} 
```



### α-β坐标系->自然坐标系(反Clark变换)

$$
\begin{bmatrix}f_A&f_B&f_C\end{bmatrix}^T=T_{3S/2S}\begin{bmatrix}f_\alpha&f_\beta&f_0\end{bmatrix}^T
$$

T<sub>2s/3s</sub>为坐标变换矩阵


$$
T_{2S/3S}=T^{-1}_{3S/2S}=\begin{bmatrix} 1 & 0 & \frac{\sqrt{2}}{2} \\\\ -\frac{1}{2} & \frac{\sqrt{3}}{2} & -\frac{\sqrt{2}}{2} \\\\ -\frac{1}{2} & -\frac{\sqrt{3}}{2} &\frac{\sqrt{2}}{2} \end{bmatrix}
$$


<img src=".\README.assets\image-20221205095704804.png" alt="image-20221205095704804" style="zoom:67%;" />

```c
static void Anti_Clark_cal(void)
{
  Anti_Clark.A = Anti_Clark.Alpha;
  Anti_Clark.B = -0.5 * Anti_Clark.Alpha + 0.8660254037844386 * Anti_Clark.Beta;
  Anti_Clark.C = -0.5 * Anti_Clark.Alpha - 0.8660254037844386 * Anti_Clark.Beta;
}
```



## 2.Park变换

### α-β坐标系->d-q坐标系（Park变换）

$$
\begin{bmatrix}f_d&f_q\end{bmatrix}^T=T_{2s/2r}\begin{bmatrix}f_\alpha&f_\beta\end{bmatrix}^T
$$

$$
T_{2S/2r}=\begin{bmatrix} cos\theta_e & sin\theta_e \\\\ -sin\theta_e & cos\theta_e \end{bmatrix}
$$

<img src=".\README.assets\image-20221205101922158.png" alt="image-20221205101922158" style="zoom:67%;" />

```c
void Plark_cal(void)
{
  Plark.D = Plark.Alpha * cos(Plark.The) + Plark.Beta * sin(Plark.The);
  Plark.Q = -Plark.Alpha * sin(Plark.The) + Plark.Beta * cos(Plark.The);
}
```

### d-q坐标系->α-β坐标系（反Park变换）

$$
\begin{bmatrix}f_\alpha&f_\beta\end{bmatrix}^T=T_{2r/2s}\begin{bmatrix}f_d&f_q\end{bmatrix}^T
$$

$$
T_{2r/2s}=T^{-1}_{2s/2r}=\begin{bmatrix} cos\theta_e & -sin\theta_e \\\\ sin\theta_e & cos\theta_e \end{bmatrix}
$$

<img src=".\README.assets\image-20221205102139385.png" alt="image-20221205102139385" style="zoom:67%;" />

```c
void Anti_Park_cal(void)
{
  Anti_Park.Ualpha = cos(Anti_Park.The) * Anti_Park.Vd - sin(Anti_Park.The) * Anti_Park.Vq;
  Anti_Park.Ubeta = sin(Anti_Park.The) * Anti_Park.Vd + cos(Anti_Park.The) * Anti_Park.Vq;
}
```



## 3.内置式三相PMSM电机方程（d-q坐标系）

定子电压方程


$$
\begin{cases}
u_d=Ri_d+\frac{d}{dt}\psi_d-\omega_e\psi_q\\
u_q=Ri_d+\frac{d}{dt}\psi_q+\omega_e\psi_d
\end{cases}
$$


定子磁链方程


$$
\begin{cases}
\psi_d=L_d+\psi_f\\
\psi_q=L_qi_q
\end{cases}
$$


定子电压方程


$$
\begin{cases}
u_d=Ri_d+L_d\frac{d}{dt}i_d-\omega_eL_qi_q\\
u_q=Ri_d+L_q\frac{d}{dt}i_q+\omega_e(L_di_q+\psi_f)
\end{cases}
$$

$$
u_d、u_q-定子电压的d-q轴分量；i_d、i_q-定子电流的d-q轴分量；\\
R-定子的电阻；\psi_d、\psi_q-定子磁链的d-q轴分量；\omega_e-电角速度；\\
L_d、L_q-d-q轴电感电量；\psi_f-永磁体磁链
$$



电磁转矩方程


$$
T_e=\frac{3}{2}p_ni_q[i_d(L_d-L_q)+\psi_f]
$$


关系式


$$
\begin{cases}
\omega_e=n_p\omega_m\\
N_r=\frac{30}{\pi}\omega_m\\
\theta_e=\int\omega_edt
\end{cases}
$$

$$
\omega_m-电机机械角度rad/s；N_r-电机转速r/min
$$



## 4.三相电压矢量表示

假设三相对称正弦相电压的瞬时值为


$$
\begin{cases}
u_a=U_msin\omega t\\
u_b=U_msin(\omega t-\frac{2}{3}\pi)\\
u_c=U_msin(\omega t+\frac{2}{3}\pi)
\end{cases}
$$


$$
U_m-相电压的幅值；\omega=2\pi f-相电压的角频率
$$


$$
U_{out}=u_a+au_b+a^2u_c\\\\
\begin{cases}
Re~U_{out}=u_a+u_bcos\frac{2}{3}\pi+u_ccos(-\frac{2}{3}\pi)=\frac{3}{2}U_msin\omega t\\
Im~U_{out}=u_bsin\frac{2}{3}\pi+u_csin(-\frac{2}{3}\pi)=-\frac{3}{2}U_mcos\omega t
\end{cases}\\\\
U_{out}=Re~U_{out}+jIm~U_{out}=\frac{3}{2}U_me^{j(\omega t-\frac{\pi}{2})}
$$


<img src=".\README.assets\image-20221205115404283.png" alt="image-20221205115404283" style="zoom:67%;" />

$$
U_{out}=\frac{2U_{dc}}{3}(s_a+s_be^{-j\frac{2}{3}\pi}+s_ce^{-j\frac{2}{3}\pi})\\\\
\begin{cases}
V_{AN}=\frac{U_{dc}}{3}(2s_a-s_b-s_c)\\\\
V_{BN}=\frac{U_{dc}}{3}(2s_b-s_a-s_c)\\\\
V_{CN}=\frac{U_{dc}}{3}(2s_c-s_a-s_b)
\end{cases}
$$


| s<sub>a</sub> | s<sub>b</sub> | s<sub>c</sub> |   V<sub>AN</sub>   |   V<sub>BN</sub>   |   V<sub>CN</sub>   | V<sub>ab</sub>  | V<sub>bc</sub>  | V<sub>ca</sub>  |          U<sub>out</sub>           |
| :-----------: | :-----------: | :-----------: | :----------------: | :----------------: | :----------------: | :-------------: | :-------------: | :-------------: | :--------------------------------: |
|       0       |       0       |       0       |         0          |         0          |         0          |        0        |        0        |        0        |                 0                  |
|       1       |       0       |       0       | 2U<sub>dc</sub>/3  | -U<sub>dc</sub>/3  | -U<sub>dc</sub>/3  | U<sub>dc</sub>  |        0        | -U<sub>dc</sub> |         2/3U<sub>dc</sub>          |
|       0       |       1       |       0       | -U<sub>dc</sub>/3  | 2U<sub>dc</sub>/3  | -U<sub>dc</sub>/3  | -U<sub>dc</sub> | U<sub>dc</sub>  |        0        | 2/3U<sub>dc</sub>e<sup>j2π/3</sup> |
|       1       |       1       |       0       |  U<sub>dc</sub>/3  |  U<sub>dc</sub>/3  | -2U<sub>dc</sub>/3 |        0        | U<sub>dc</sub>  | -U<sub>dc</sub> | 2/3U<sub>dc</sub>e<sup>jπ/3</sup>  |
|       0       |       0       |       1       | -U<sub>dc</sub>/3  | -U<sub>dc</sub>/3  | 2U<sub>dc</sub>/3  |        0        | -U<sub>dc</sub> | U<sub>dc</sub>  | 2/3U<sub>dc</sub>e<sup>j4π/3</sup> |
|       1       |       0       |       1       |  U<sub>dc</sub>/3  | -2U<sub>dc</sub>/3 |  U<sub>dc</sub>/3  | U<sub>dc</sub>  | -U<sub>dc</sub> |        0        | 2/3U<sub>dc</sub>e<sup>j5π/3</sup> |
|       0       |       1       |       1       | -2U<sub>dc</sub>/3 |  U<sub>dc</sub>/3  |  U<sub>dc</sub>/3  | -U<sub>dc</sub> |        0        | U<sub>dc</sub>  |  2/3U<sub>dc</sub>e<sup>jπ</sup>   |
|       1       |       1       |       1       |         0          |         0          |         0          |        0        |        0        |        0        |                 0                  |

<img src=".\README.assets\image-20221205180755310.png" alt="image-20221205180755310" style="zoom:67%;" />



## 5.SVPWM原理

平衡等效原则：在一个开关周期T<sub>s</sub>内通过对基本电压矢量加以组合，使其平均值与给定电压矢量相等。

以扇区Ⅰ为例：

$$
T_sU_{out}=T_4U_4+T_6U_6+T_0(U_0或U_7)\\
T_4+T_6+T_0=T_S\\
\begin{cases}
U_1=\frac{T_4}{T_s}U_4\\
U_2=\frac{T_6}{T_s}U_6
\end{cases}\\
T_4,T_6,T_0-U_4,U_6和零矢量U_0或U_7的作用时间
$$

![image-20221205204109307](.\README.assets\image-20221205204109307.png)
$$
|U_4|=|U_6|=\frac{2}{3}U_{dc}\\
|U_{out}|=U_m\\
\begin{cases}
T_4=\sqrt{3}\frac{U_m}{U_{dc}}T_ssin(\frac{\pi}{3}-\theta)\\
T_6=\sqrt{3}\frac{U_m}{U_{dc}}T_ssin\theta\\
T_0=T_7=\frac{1}{2}(T_s-T_4-T_6)
\end{cases}\\
SVPWM调制比M=\frac{\sqrt{3}U_m}{U_{dc}}\\
|U_{out}|=U_m\leqslant\frac{2U_{dc}}{3},即M_{max}=\frac{2}{\sqrt{3}}=1.1547
$$

SVPWM调制中，调制深度最大值可以达到1.1547，比SPWM调制最高所能达到的调制比1高0.1547，这使其直流母线电压利用率更高。

| U<sub>out</sub>所在位置 |  开关切换顺序   |                          三相波形图                          |
| :---------------------: | :-------------: | :----------------------------------------------------------: |
|      Ⅰ区(0°≤θ≤60°)      | 0→4→6→7→7→6→4→0 | <img src=".\README.assets\image-20221205214059385.png" alt="image-20221205214059385" style="zoom: 67%;" /> |
|     Ⅱ区(60°≤θ≤120°)     | 0→2→6→7→7→6→2→0 | <img src=".\README.assets\image-20221205214145721.png" alt="image-20221205214145721" style="zoom:67%;" /> |
|    Ⅲ区(120°≤θ≤180°)     | 0→2→3→7→7→3→2→0 | <img src=".\README.assets\image-20221205214218434.png" alt="image-20221205214218434" style="zoom:67%;" /> |
|    Ⅳ区(180°≤θ≤240°)     | 0→1→3→7→7→3→1→0 | <img src=".\README.assets\image-20221205214236690.png" alt="image-20221205214236690" style="zoom:67%;" /> |
|    Ⅴ区(240°≤θ≤300°)     | 0→1→5→7→7→5→1→0 | <img src=".\README.assets\image-20221205214255778.png" alt="image-20221205214255778" style="zoom:67%;" /> |
|    Ⅵ区(300°≤θ≤360°)     | 0→4→5→7→7→5→4→0 | <img src=".\README.assets\image-20221205214311499.png" alt="image-20221205214311499" style="zoom:67%;" /> |



## 6.SVPWM实现

### 扇区判断

判断电压空间矢量U<sub>out</sub>所在扇区的目的是确定本开关周期所使用的基本电压空间矢量。用u<sub>α</sub>和u<sub>β</sub>表示参考电压矢量U<sub>out</sub>在α-β轴上的分量，定义U<sub>ref1</sub>、U<sub>ref2</sub>和U<sub>ref3</sub>三个变量。

$$
\begin{cases}
U_{ref1}=u_\beta\\
U_{ref2}=\frac{\sqrt{3}}{2}u_\alpha-\frac{1}{2}u_\beta\\
U_{ref3}=-\frac{\sqrt{3}}{2}u_\alpha-\frac{1}{2}u_\beta
\end{cases}
$$

再定义三个变量A、B、C，通过分析可以得出：

若U<sub>ref1</sub>>0，则A=1，否则A=0；

若U<sub>ref2</sub>>0，则B=1，否则B=0；

若U<sub>ref3</sub>>0，则C=1，否则C=0；

令N=4C+2B+A

|  N   |  3   |  1   |  5   |  4   |  6   |  2   |
| :--: | :--: | :--: | :--: | :--: | :--: | :--: |
| 扇区 |  Ⅰ   |  Ⅱ   |  Ⅲ   |  Ⅳ   |  Ⅴ   |  Ⅵ   |

<img src=".\README.assets\image-20221205221336870.png" alt="image-20221205221336870" style="zoom:67%;" />

<img src=".\README.assets\image-20221206121314617.png" alt="image-20221206121314617" style="zoom: 67%;" />

```c
void Sector_cal(void)
{
  Sector.N = ((-Sector.Ualpha * 0.8660254037844386 - 0.5 * Sector.Ubeta >= 0.0) * (int32_T)4.0 + (Sector.Ualpha *  0.8660254037844386 - 0.5 * Sector.Ubeta >= 0.0) * (int32_T)2.0) + (Sector.Ubeta >= 0.0);
}
```



 ### 矢量作用时间



![image-20221206102731904](.\README.assets\image-20221206102731904.png)


$$
\begin{cases}
u_\alpha=\frac{T_4}{T_s}|U_4|+\frac{T_6}{T_s}|U_6|cos\frac{\pi}{3}\\
u_\beta=\frac{T_6}{T_s}|U_6|sin\frac{\pi}{3}
\end{cases}
\Longrightarrow
\begin{cases}
T_4=\frac{\sqrt{3}T_s}{2U_{dc}}(\sqrt{3}u_\alpha-u_\beta)\\
T_6=\frac{\sqrt{3}T_s}{2U_{dc}}u_\beta
\end{cases}
$$

同理，分析其他扇区各矢量作用时间，可得：

$$
\begin{cases}
X=\frac{\sqrt{3}T_sU_\beta}{U_{dc}}\\
Y=\frac{\sqrt{3}T_s}{2U_{dc}}(\sqrt{3}u_\alpha+u_\beta)\\
Z=\frac{\sqrt{3}T_s}{2U_{dc}}(-\sqrt{3}u_\alpha+u_\beta)
\end{cases}
$$

<img src=".\README.assets\image-20221206121352481.png" alt="image-20221206121352481" style="zoom:67%;" />


| 扇区          | Ⅰ    | Ⅱ    | Ⅲ    | Ⅳ    | Ⅴ    | Ⅵ    |
| ------------- | ---- | ---- | ---- | ---- | ---- | ---- |
| N             | 3    | 1    | 5    | 4    | 6    | 2    |
| T<sub>1</sub> | 0    | 0    | 0    | -X   | -Y   | 0    |
| T<sub>2</sub> | 0    | Z    | X    | 0    | 0    | 0    |
| T<sub>3</sub> | 0    | 0    | -Y   | Z    | 0    | 0    |
| T<sub>4</sub> | -Z   | 0    | 0    | 0    | 0    | Y    |
| T<sub>5</sub> | 0    | 0    | 0    | 0    | -Z   | -X   |
| T<sub>6</sub> | X    | Y    | 0    | 0    | 0    | 0    |

设任意扇区中两个非零矢量作用时间为T<sup>'</sup>和T<sup>''</sup>

| T<sub>0</sub>(T<sub>7</sub>) | (T<sub>s</sub>-T<sup>'</sup>-T<sup>''</sup>)/2 |
| --------------- | ------------------------ |

```c
void XYZ_step(void)
{
  XYZ.X = 1.7320508075688772 * XYZ.Ubeta * 1.0 / XYZ.Udc * Ts;
  XYZ.Y = (0.8660254037844386 * XYZ.Ubeta + 1.5 * XYZ.Ualpha) * 1.0 / XYZ.Udc * Ts;
  XYZ.Z = (0.8660254037844386 * XYZ.Ubeta - 1.5 * XYZ.Ualpha) * 1.0 / XYZ.Udc * Ts;
}
```

如果T<sup>'</sup>+T<sup>''</sup> > T<sub>s</sub>，则需进行过调制处理：



$$
\begin{cases}
T^{'}=\frac{T^{'}}{T^{'}+T^{''}}T_s\\
T^{''}=\frac{T^{''}}{T^{'}+T^{''}}T_s
\end{cases}
$$



<img src=".\README.assets\image-20221206121420940.png" alt="image-20221206121420940" style="zoom: 50%;" />



```c
void T1T2_cal(void)
{
  switch ((int32_T)Sector.N) {
   case 1:
    T1T2.T1 = XYZ.Z;
    T1T2.T2 = XYZ.Y;
    break;
   case 2:
    T1T2.T1 = XYZ.Y;
    T1T2.T2 = -XYZ.X;
    break;
   case 3:
    T1T2.T1 = -XYZ.Z;
    T1T2.T2 = XYZ.X;
    break;
   case 4:
    T1T2.T1 = -XYZ.X;
    T1T2.T2 = XYZ.Z;
    break;
   case 5:
    T1T2.T1 = XYZ.X;
    T1T2.T2 = -XYZ.Y;
    break;
   default:
    T1T2.T1 = -XYZ.Y;
    T1T2.T2 = -XYZ.Z;
    break;
  }

  if ((T1T2.Tpwm - T1T2.T2 - T1T2.T1) < 0.0) {
    T1T2.T1 = T1T2.T1 * T1T2.Tpwm / (T1T2.T1 + T1T2.T2);
    T1T2.T2 = T1T2.T2 * T1T2.Tpwm / (T1T2.T1 + T1T2.T2);
  }
}
```



### 矢量切换点

定义

$$
\begin{cases}
T_a=(T_s-T^{'}-T^{''})/4\\
T_b=T_a+T^{'}/2\\
T_c=T_b+T^{''}/2
\end{cases}
$$

| 扇区            | Ⅰ             | Ⅱ             | Ⅲ             | Ⅳ             | Ⅴ             | Ⅵ             |
| --------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| N               | 3             | 1             | 5             | 4             | 6             | 2             |
| T<sub>cm1</sub> | T<sub>a</sub> | T<sub>b</sub> | T<sub>c</sub> | T<sub>c</sub> | T<sub>b</sub> | T<sub>a</sub> |
| T<sub>cm2</sub> | T<sub>b</sub> | T<sub>a</sub> | T<sub>a</sub> | T<sub>b</sub> | T<sub>c</sub> | T<sub>c</sub> |
| T<sub>cm3</sub> | T<sub>c</sub> | T<sub>c</sub> | T<sub>b</sub> | T<sub>a</sub> | T<sub>a</sub> | T<sub>b</sub> |

<img src=".\README.assets\image-20221206121452363.png" alt="image-20221206121452363" style="zoom: 67%;" />

```c
void Tcm_cal(void)
{
    Tcm.Ta = (Tcm.Tpwm - Tcm.T1 - Tcm.T2) / 4.0;
    Tcm.Tb = (Tcm.Tpwm + Tcm.T1 - Tcm.T2) / 4.0;
    Tcm.Tc = (Tcm.Tpwm + Tcm.T1 + Tcm.T2) / 4.0;
  switch ((int32_T)Sector.N) {
   case 1:
    Tcm.Tcm1 = Tcm.Tb;
    Tcm.Tcm2 = Tcm.Ta;
    Tcm.Tcm3 = Tcm.Tc;
    break;
   case 2:
    Tcm.Tcm1 = Tcm.Ta;
    Tcm.Tcm2 = Tcm.Tc;
    Tcm.Tcm3 = Tcm.Tb;
    break;
   case 3:
    Tcm.Tcm1 = Tcm.Ta;
    Tcm.Tcm2 = Tcm.Tb;
    Tcm.Tcm3 = Tcm.Tc;
    break;
   case 4:
    Tcm.Tcm1 = Tcm.Tc;
    Tcm.Tcm2 = Tcm.Tb;
    Tcm.Tcm3 = Tcm.Ta;
    break;
   case 5:
    Tcm.Tcm1 = Tcm.Tc;
    Tcm.Tcm2 = Tcm.Ta;
    Tcm.Tcm3 = Tcm.Tb;
    break;
   default:
    Tcm.Tcm1 = Tcm.Tb;
    Tcm.Tcm2 = Tcm.Tc;
    Tcm.Tcm3 = Tcm.Ta;
    break;
  }
}
```



## 7.PI调节器参数整定

### 转速环

三相PMSM电机运动方程：


$$
\begin{cases}
J\frac{d\omega_m}{dt}=T_e-T_L-B\omega_m\\
T_e=\frac{3}{2}p_ni_d[i_d(L_d-L_q)+\psi_f]
\end{cases}
$$


ω<sub>m</sub>-电机的机械角速度；J-转动惯量；B-阻尼系数；T<sub>L</sub>-负载转矩

定义有功阻尼：

$$
i_q=i^`_q-B_a\omega_m
$$

采用i<sup>*</sup><sub>d</sub>=0的控制策略，并假定电机在空载(T<sub>L</sub>=0)情况下启动：

$$
\frac{d\omega_m}{dt}=\frac{1.5p_n\psi_f}{J}(i^`_q-B_a\omega_m)-\frac{B}{J}\omega_m
$$

将上式的极点配置到期望的闭环带宽β，可以得到转速相对于q轴电流的传递函数为：

$$
\omega_m(s)=\frac{1.5p_n\psi_f/J}{s+\beta}i^`_q(s)
$$

得有功系数B<sub>a</sub>：

$$
B_a=\frac{\beta J-B}{1.5p_n\psi_f}
$$

若采用传统的PI调节器，则转速环控制器的表达式：

$$
i^*_q=(K_{p\omega}+\frac{K_{i\omega}}{s})(\omega^*_m-\omega_m)-B_a\omega_m\\
\Longrightarrow
\begin{cases}
K_{p\omega}=\frac{\beta J}{1.5p_n\psi_f}\\
K_{i\omega}=\beta K_{p\omega}
\end{cases}
$$



![image-20221206175333122](.\README.assets\image-20221206175333122.png)




```c
void Speed_PI(void)
{

  Speed.Out = (Speed.Kp * Speed.Error + Speed.DTI) - Speed.Ba * Speed.Error;

  if (Speed.Out > Speed.UpperLimiter) {
    Speed.Out = Speed.UpperLimiter;
  } else if (Speed.Out < Speed.LowerLimiter) {
    Speed.Out = Speed.LowerLimiter;
  }

  Speed.DTI += Speed.Ki * Speed.Error * Ts;
    
  if (Speed.DTI >= Speed.UpperLimiter) {
    Speed.DTI = Speed.UpperLimiter;
  } else if (Speed.DTI <= Speed.LowerLimiter) {
    Speed.DTI = Speed.LowerLimiter;
  }
}
```



### 电流环

重写d-q坐标系下的电流方程：

$$
\begin{cases}
\frac{d}{dt}i_d=-\frac{R}{L_d}i_d+\frac{L_q}{L_d}\omega_ei_q+\frac{1}{L_d}u_d\\
\frac{d}{dt}i_q=-\frac{R}{L_d}i_d-\frac{1}{L_q}\omega_e(L_di_d+\psi_f)+\frac{1}{L_q}u_q
\end{cases}
$$

定子电流i<sub>d</sub>，i<sub>q</sub>分别在q轴和d轴方向产生交叉耦合电动势。

若i<sub>d</sub>，i<sub>q</sub>完全解耦，可得：

$$
\begin{cases}
u_{d0}=u_d+\omega_eL_qi_q=Ri_d+L_d\frac{d}{dt}i_d\\
u_{q0}=u_q-\omega_e(L_di_d+\psi_f)=Ri_q+L_q\frac{d}{dt}i_q
\end{cases}
$$

其中，u<sub>d0</sub>和u<sub>q0</sub>分别为电流解耦后的d轴和q轴电压

对上式进行拉普拉斯变换：

$$
\pmb{Y}(s)=\pmb{G}(s)\pmb{U}(s)\\
\pmb{U}(s)=\begin{bmatrix}u_{d0}(s)\\\\u_{q0}(s)\end{bmatrix},\pmb{Y}(s)=\begin{bmatrix}i_d(s)\\\\i_q(s)\end{bmatrix},\pmb{G}(s)=\begin{bmatrix}R+sL_d&0\\\\0&R+sL_q\end{bmatrix}^{-1}
$$

采用常规的PI调节器并结合前馈解耦控制策略，可得到d-q轴的电压：

$$
\begin{cases}
v^*_d=(K_{pd}+\frac{K_{id}}{s})(i^*_d-i_d)-\omega_eL_qi_q\\
u^*_q=(K_{pq}+\frac{K_{id}}{s})(i^*_q-i_q)+\omega_e(L_di_d+\psi_f)
\end{cases}
$$

其中，K<sub>pd</sub>和K<sub>pd</sub>为控制器的比例增益，K<sub>id</sub>和K<sub>id</sub>为PI控制器的控制增益

内模控制框图：

$$
\pmb{G}(s)为内模，\pmb{G}(s)为被控对象，\pmb{C}(s)为内模控制器
$$



![image-20221206202847141](.\README.assets\image-20221206202847141.png)



其等效控制器为：
$$
\pmb{F}(s)=[\pmb{I}-\pmb{C}(s)\hat{\pmb{G}}(s)]^{-1}\pmb{C}(s)
$$

如果内模建模精确，即

$$
\pmb{\hat{G}}(s)=\pmb{G}(s)
$$

则系统不存在反馈环节，此时系统传递函数为：

$$
\pmb{G}_c(s)=\pmb{G}(s)\pmb{C}(s)
$$

要保证系统稳定，当且只有当**G**(s)和**C**(s)稳定

由于电机的电磁时间常数比机械时间常数小得多，控制系统的电流环可近似看作一阶系统

$$
根据\pmb{\hat{G}}(s)=\pmb{G}(s)，定义\pmb{C}(s)=\pmb{\hat{G}}^{-1}(s)\pmb{L}(s)=\pmb{G}^{-1}(s)\pmb{L}(s)\\
其中：\pmb{L}(s)=\alpha\pmb{I}(s+\alpha)，\alpha为设计参数
$$

内模控制器：

$$
\pmb{F}(s)=\alpha\begin{bmatrix}L_d+\frac{R}{s}&0\\\\0&L_q+\frac{R}{s}\end{bmatrix}\\
\pmb{G}_c(s)=\frac{\alpha}{s+\alpha}\pmb{I}
$$

$$
\begin{cases}
K_{pd}=\alpha L_d\\
K_{id}=\alpha R\\
K_{pq}=\alpha L_q\\
K_{iq}=\alpha R
\end{cases}
$$

定义响应时间t<sub>res</sub>为系统响应从阶跃的10%~90%所需的时间，则α与t<sub>res</sub>的关系近似为t<sub>res</sub>=ln9/α。由α与t<sub>res</sub>的关系可知，减小α将延长系统响应时间，增大α将加快系统响应速度，但α不能无限增大，实际中系统响应时间受电气时间常数的限制，电机的时间常数为：

$$
\begin{cases}
T_d=\frac{L_d}{R}\\
T_q=\frac{L_q}{R}
\end{cases}
$$


<img src=".\README.assets\image-20221206212947960.png" alt="image-20221206212947960" style="zoom:67%;" />


```c
void Iq_PI(void)
{
  Iq.Out = Iq.Kp * Iq.Error + Iq.DTI;

  if (Iq.Out > Iq.UpperLimit) {
    Iq.Out = Iq.UpperLimit;
  } else if (Iq.Out < Iq.LowerLimit) {
    Iq.Out = -Iq.LowerLimit;
  }

  Iq.DTI += Iq.Ki * Iq.Error * Ts;
  if (Iq.DTI >= Iq.UpperLimit) {
    Iq.DTI = Iq.UpperLimit;
  } else if (Iq.DTI <= Iq.LowerLimit) {
    Iq.DTI = Iq.LowerLimit;
  }
}
```

```c
void Id_PI(void)
{
  Id.Out = Id.Kp * Id.Error + Id.DTI;

  if (Id.Out > Id.UpperLimit) {
    Id.Out = Id.UpperLimit;
  } else if (Id.Out < Id.LowerLimit) {
    Id.Out = Id.LowerLimit;
  }

  Id.DTI += Id.Ki * Id.Error * Ts;
  if (Id.DTI >= Id.UpperLimit) {
    Id.DTI = Id.UpperLimit;
  } else if (Id.DTI <= Id.LowerLimit) {
    Id.DTI = Id.LowerLimit;
  }
}

```

