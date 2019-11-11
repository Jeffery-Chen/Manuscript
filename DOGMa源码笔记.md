# DOGMa源码笔记

`Github`源码出处[mitkina/dogma](mitkina/dogma)

`arxiv`论文出处[A Random Finite Set Approach for Dynamic Occupancy Grid Maps with Real-Time Application](https://arxiv.org/abs/1605.02406)

## Simulator

### `def create_data`

生成一个二维向量*n_elems × n_elems*存储地图信息

选用`xrange()`不会在内存里创建列表的完整拷贝，而是一个可迭代对象，在`for`循环之外没有任何意义，多用于超大数据集

### `class Grid`

表示全局环境栅格

#### `def __init__ `

参数*vehicles, w=256, dx=1, counter=0*将传入到该方法

#### `def place_vehicles`

在栅格地图*create_data*中绘制（置1）仿真用车辆

#### `def get_vehicle_cells`

传入车辆的中心位置以及长宽信息，返回对应的占用栅格位置

- Bounding box approximation “膨胀原则”

  > `shape[0],shape[1]`与`w,h`的对应关系

- Map to grid indices

- Fill all encompassed indices

#### `def grid_floor`

使用`np.floor`对栅格值向下取整

#### `def grid_ceil`

使用`np.ceil`对栅格值向上取整

#### `def pos_to_ind`

基于栅格空间的精确坐标值索引，返回整型

#### `def create_global_grid`

使用`np.meshgrid`建立栅格坐标系(x,y)，`sparse=True`开启稀疏栅格以减小内存占用

```python
self.global_x_grid = gridx.T	'''转置后用于pickle.dump序列化对象，默认为ASCⅡ协议'''
self.global_y_grid = gridy.T
```

#### `def tick`

使所有车辆运动并绘制

```python
import matplotlib.pyplot as plt
vehicle.move()	'''self.pos += self.vel * dt'''
plt.imshow(X,cmap,interpolation)	'''X:image,cmap:colormap,interplation:none对图像进行处理而不显示'''
os.path.join(path,*path)	'''获取路径path并与*path组成新路径'''
plt.show()	'''紧跟plt.imshow对图像进行显示'''
```

### `class Mover`

定义对象运动学的通用接口

#### `def __init__`

`np.array`生成单一类型的多维数组，而`list`需要生成所需的多个指针与数据对象

#### `def move`

> - [ ] 此处可为所有基于Mover类的运动对象添加运动噪声

#### `def get_heading`

$$
\Theta = \frac {V_y} {V_x}
$$

### `class Car(Mover)`

定义`car`的尺寸`CAR_DIMENS`为 $5 \times 10$ 

`super()`用于调用父类（超类）的一个方法，以解决多重继承问题

```python
def __init__(self, pos, vel):
    super(Car, self).__init__(pos, vel, Car.CAR_DIMENS)
    '''找到父类Mover，将子类Car的对象转换为父类Mover的对象'''
```

### `class Pedestrian(Mover)`

定义`pedestrian`的尺寸`PED_DIMENS`为 $4 \times 4$ 

### `def main()`

`np.random.seed(seed)`用于指定生成随机数所用算法的起始值，且一次有效，若不继续设置则会根据系统时间自行选择

`__name__ == '__main__'`是python的主函数入口，用于判断当前是否是被python直接调用执行



## Grid

### `class GridCell`

表征单个占用栅格单元信息

### `class CellArray`

通用数组类

- `C`：占用栅格总数

```python
getattr(self.cells[cell_index],attribute)	'''获取对象的属性值attribute'''
setattr(self.cells[cell_index],attribute,value)	'''设置属性值，该属性不必存在'''
```

### `class GridCellArray(CellArray)`

表征（全局）栅格数组

### `class MeasCellArray(CellArray)`

表征测量栅格数组

- `pseudoG = 1.`——对应式（69）中的**spatial measurement likelihood** $g ^{(c)} _{k+1} (z _{k+1} | x_{k+1})$，用于非归一化的权重更新
- 下标记号`+`表征预测，相当于`k+1|k`

$$
\tilde{\omega} ^{(i,c)} _{p,k+1} = 
	g ^{(c)} _{k+1} (z _{k+1}|x ^{(i,c)} _{p,+}) \cdot 
	\omega ^{(i,c)} _{p,+}
\tag{69}
$$



## Particle

```python
import pdb	'''为python自带包，提供了一种交互的源代码调试功能'''
```

### `class Particle`

表征单个粒子信息

`state_size = 4, state = np.zeros([state_size])`，标准差为`scale = self.scale_vel = 5.`

#### `def predict`

定常速度模型（**constant velocity model**，一种用以定义**transition density**的处理模型）引自式（14），$\zeta_k$为处理噪声，$T$为$k$与$k+1$之间的时间间隔

`noiseSD = 0.1`
$$
x_{k+1} = 
			\begin{pmatrix} 1 & 0 & T & 0 \\
						  0 & 1 & 0 & T \\
						  0 & 0 & 1 & 0 \\
						  0 & 0 & 0 & 1 \\
			\end{pmatrix}
			x_k + \zeta_k \tag {14}
$$

```python
np.dot(a,b)		'''矩阵相乘'''
c = a*b			'''对应元素相乘'''
```

#### `def persist_update_weight`

引自文中式（39）更新原有粒子权重
$$
\omega ^{(i)} _{p,+} = p _{S} \cdot \omega ^{(i)} _{k}
\tag{39}
$$

#### `def normalize`

原理引自文中式（71）归一化粒子权重
$$
\omega ^{(i,c)} _{p,k+1} = p ^{(c)} _{A,k+1} \cdot \mu ^{(c)} _{A} \cdot 							\tilde{\omega} ^{(i,c)} _{p,k+1} + \left( 1-p ^{(c)} _{A,k+1}\right)                            \cdot \mu ^{(c)} _{\overline{A}} \cdot                            \omega ^{(i,c)} _{p,+}\tag{71}
$$

其中 $\mu ^{(c)} _{A}$ 与 $\mu ^{(c)} _{\overline{A}}$ 为已知输入量
$$
\mu ^{(c)} _{A} = {\left[ \sum ^{v ^{(c)} _{p,+}} _{i=1} \tilde{\omega} ^{(i,c)} _{p,k+1} \right]} ^{-1} \cdot \rho ^{(c)} _{p,k+1}\tag{72}
$$

$$
\mu ^{(c)} _{\overline{A}} = \left[ \sum ^{v ^{(c)} _{p,+}} _{i=1} \omega ^{(i,c)} _{p,+} \right] ^{-1} \cdot \rho ^{(c)} _{p,k+1} = \frac{\rho ^{(c)} _{p,k+1}}{m ^{(c)} _{p,+}(O _{k+1})}\tag {73}
$$

### `class ParticleArray`

表征全部粒子的列表（`nu=2*10**6`连续粒子个数，`T=0.1`时间间隔，`transition_matrix`采用式（14）中的变换矩阵）

````python
if ... :
	pass
else:
    raise Exception("抛出一个异常")
````

> - [ ] 在网格中心初始化粒子

```python
for i in range(nu):
    index = np.random.choice(C)
	# initializing particles at the center of cells
    x = index % X + 0.5 # operators were switched before
    y = index / X + 0.5
```

### `def initialize_newborn_A`

原理引自式（74），用于初始化关联的新生粒子，目前使用的是一个**虚拟分布（dummy distribution）**，应用时还需改为实际服从的分布
$$
x ^{(i,c)} _{A,k+1} \sim p ^{(c)} _{x _{k+1}} (\cdot | z ^{(c)} _{k+1})
\tag{74}
$$

### `def initialize_newborn_UA`

原理引自式（76）初始化无关联的新生粒子，目前使用的是一个**虚拟分布（dummy distribution）**，应用时还需改为实际服从的分布
$$
x ^{(i,c)} _{\overline{A},k+1} \sim b _{k+1}(\cdot)
\tag{76}
$$



## 0. Particle Filter Functions

模拟环境为与本车对向的两辆车辆以**10m/s**的速度驶来，栅格地图分辨率为**33cm**

- 具体参数设置如下

|       参数       |              描述              |      值       |
| :--------------: | :----------------------------: | :-----------: |
|      $p\_B$      |            出生概率            |     0.02      |
|       $Vb$       |           新生粒子数           | $2\times10^4$ |
|       $V$        |           原有粒子数           | $2\times10^5$ |
|  $state\_size$   |          $p,v$ 状态数          |       4       |
|     $alpha$      |          信息衰变因数          |      0.9      |
|      $p\_A$      | 关联概率（仅与多普勒测量有关） |      1.0      |
|       $T$        |        测量频率（10Hz）        |      0.1      |
|      $p\_S$      |          粒子保留概率          |     0.99      |
|      $res$       |           栅格分辨率           |      1.0      |
|   $scale\_vel$   |          初始速度方差          |     12.0      |
|   $scale\_acc$   |         初始加速度方差         |      2.0      |
|  $process\_pos$  |            位置噪声            |     0.06      |
|  $process\_vel$  |            速度噪声            |      2.4      |
|  $process\_acc$  |           加速度噪声           |      0.2      |
|    $verbose$     |        是否打印debug值         |     False     |
|       $mS$       |            画图阈值            |      3.0      |
|    $epsilon$     |              ...               |     10.0      |
|  $epsilon\_occ$  |              ...               |     0.75      |
| $index\_stopped$ |        粒子滤波中断索引        |       0       |

- `get_dogma`保存动态栅格地图 $4\times128\times128$ 

```python
import numpy as np
newDOGMA = np.stack((posO, posF, velX, velY, meas_grid))
'''用以连接数组，但要求数组形状相同'''
```

- `dogma2head_grid`为绘图工具创建**heading grid**

## 1. Particle Prediction

遍历粒子数组，更新粒子权重，核心是调用`ParticleArray`类下的`persist_update_weight`方法

## 2. Assignment of Particles to Grid Cells

将生成的随机粒子分配给栅格单元，核心是调用`CellArray`类下的`set_cell_attr`方法

## 3. Grid Cell Occupancy Prediction and Update

为每一个栅格更新其对应的**Dempster-Shafer**概率密度函数值

- 调用`Particle_Array`类下的`accumulate_weight`方法，累计粒子权重
- 调用`Particle_Array`类下的`normalize_p_S`方法，以归一化超出阈值的粒子权重
- 预测空闲栅格的概率密度（与静态地图相同），引自式（62）

$$
m ^{(c)} _{p,+} (F _{k+1}) = min \left[ 
									\alpha (T) m ^{(c)} _k (F _k) , 1 - m ^{(c)} _{p,+} 
									(O _{k+1})
									\right]
\tag{62}
$$

- 综合测量值与预测值计算后验占用与空闲栅格概率，引自式（63）

$$
m ^{(c)} _{k+1} = m ^{(c)} _{p,+} \oplus m ^{(c)} _{z _{k+1}}
\tag{63}
$$

- 计算后验占用概率密度的新生部分，引自式（67）

$$
\rho ^{(c)} _{b,k+1} = \frac
{m ^{(c)} _{k+1} (O _{k+1}) \cdot p _{B} \left[ 1 - m ^{(c)} _{p,+}(O _{k+1}) \right]}
{m ^{(c)} _{p,+} (O _{k+1}) + p _{B} \left[ 1 - m ^{(c)} _{p,+}(O _{k+1}) \right]}
\tag{67}
$$

## 4. Update of Persistent Particles

更新原有粒子概率

- `update_unnorm`非归一化权重更新，引自式（69），详见`MeasCellArray`类

- `accumulate_weight`累计非归一化原有权重
- `calc_norm_assoc`引自式（72），`calc_norm_unassoc`引自式（73），详见`Particle`类下的`normalize`方法

## 5. Initialization of New Particles

初始化新生粒子

- `normalize_particle_orders`归一化新粒子数为定常值$v_b$
- `calc_num_assoc`计算新生关联/非关联粒子，引自式（79）（80）

$$
\frac{v ^{(c)} _{A,k+1}}{v ^{(c)} _{\overline{A},k+1}} = 
\frac{p ^{(c)} _{A,k+1}}{1-p ^{(c)} _{A,k+1}}
\tag{79}
$$

$$
v ^{(c)} _{A,k+1} + v ^{(c)} _{\overline{A},k+1} = v ^{(c)} _{b,k+1}
\tag{80}
$$



- `calc_weight_assoc`计算新生关联/非关联粒子的权重，引自式（75）（77）

$$
\omega ^{(i,c)} _{A,k+1} = \frac{p ^{(c)} _{A,k+1} \cdot \rho ^{(c)} _{b,k+1}}
{v ^{(c)} _{A,k+1}}
\tag{75}
$$

$$
\omega ^{(i,c)} _{\overline{A},k+1} = \frac{\left( 1-p^{(c)}_{A,k+1} \right) \cdot \rho^{(c)}_{b,k+1}}
{v^{(c)}_{\overline{A},k+1}}
\tag{77}
$$

- `initialize_new_particle_A` / `initialize_new_particle_UA`初始化关联/非关联粒子，引自式（74）/（76），详见`PaticleArray`类下的`initialize_newborn_A`以及`initialize_newborn_UA`方法

## 6. Statistical Moments of Grid Cells

计算各个栅格的速度（均值与方差）

- `mean_x_vel`&`mean_v_vel`计算均值，引自式（81）

$$
\overline{v}^{(c)}_{x} \approx \frac{1}{\rho^{(c)}_{p,k+1}} \sum^{v^{(c)}_{p,k+1}}_{i=1}
\omega^{(i,c)}_{p,k+1} \cdot v^{(i,c)}_{x,p,k+1}
\tag{81}
$$

- `var_x_vel`&`var_y_vel`计算方差，引自式（83）

$$
{\sigma ^{2}}^{(c)}_{v_{x}} \approx \frac{1}{\rho^{(c)}_{p,k+1}} \sum^{v^{(c)}_{p,k+1}}_{i=1}
\omega^{(i,c)}_{p,k+1} \cdot \left( v^{(i,c)}_{x,p,k+1} \right)^{2} - {\left( \overline{v}^{(c)}_{x} \right)}^{2}
\tag{83}
$$

- `covar_xy_vel`计算协方差，引自式（84）

$$
\sigma ^{(c)}_{v_{x},v_{y}} \approx \frac{1}{\rho^{(c)}_{p,k+1}} \sum^{v^{(c)}_{p,k+1}}_{i=1}
\omega^{(i,c)}_{p,k+1} \cdot v^{(i,c)}_{x,p,k+1} \cdot \overline{v}^{(i,c)}_{y,p,k+1} - \overline{v}^{(c)}_{x} \cdot \overline{v}^{(c)}_{y}
\tag{84}
$$

## 7. Resampling

在原有粒子与新生粒子集中进行重采样

- `calc_resampled_indeces`计算重采样粒子的索引角标

- [ ] > - [ ] 可能会因为**舍入误差（rounding error）**而不成立



# 附录

- `BBA` / `BPA`: Basic Belif Assignment / Basic Probability Assignment
- `BBF`: Binary Bayes Filter
- `BF`: Bernoulli Filter
- `BOF`: Bayesian Occupancy Filter
- `FISST`: Finite Set Statistics
- `MIB`: Multi-Instance Bernoulli
- `PHD`: Probability Hypothesis Density

- `RFS`: Random Finite Set
- `SLAMMOT`: Simultaneous Localization, Mapping,and Moving Object Tracking

- `SMC`: Sequential Monte Carlo