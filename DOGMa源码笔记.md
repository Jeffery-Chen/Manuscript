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

> 此处可为所有基于Mover类的运动对象添加运动噪声

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

## Particle Filter Functions

```python
import matplotlib
matplotlib.use('Agg')	'''设置matplotlib后台为Agg，如果需要修改要在导入matplotlib后第一时间设置'''
import sys
sys.path.insert(0,'..')		''''''
```

