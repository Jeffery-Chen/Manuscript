# Aibbt Python

## 环境配置

### Windows安装Python环境

进入交互环境

```python
python
```

推出交互环境

```python
exit()
```

若安装Python时没有勾选Add Python 3.x to PATH，就会出现如下错误，需手动添加环境变量或重新安装

```python
'python' is not recognized as internal or external command, operable program or batch file. 
```

Python官方版本自带解释器，不需要再次安装



## 变量与字符串

### 变量类型转换

`int()`：将数值或字符串转换成整数，可指定进制

`float()`：将字符串转换成浮点数

`str()`：将指定对象转换成字符串形式，可指定编码

`chr()`：将整型转换成该编码对应的字符串（一个字符）

`ord()`：将字符串（一个字符）转换成对应的编码（整数）

### 字符串分片（slice）与索引

==‘ ’== = ==“ ”== 

==‘’‘ ’‘’== 可换行

截取字符串中编号从第n个字符起，到编号为m的字符止，且不包含第m个字符

```python
string[n: m]
```

### 字符串的方法

`count()`方法用于统计字符串里某个字符出现的次数

```python
str.count(sub, start = 0, end = len(string))	#sub为搜索的子字符串
```

`replace()` 替换，string[n: m]即为需要替换掉的部分，’x‘则为将要替换的字符

```python
string1.replace(string2[n: m], 'x')
```

`find()`找到string2对应string1所在的索引位置

```python
string1.find(string2)
```

`format()`字符串填空

```python
string.format()

print('{} a word she can get what she {} for.'.format('With', 'came'))
print('{preposition} a word she can get what she {verb} for.'.format(preposition = 'With', verb = 'came'))
print('{0} a word she can get what she {1} for.'.format('With', 'came'))

import math
print('The value of PI is approximately {!r}.'.format(math.pi))
```

=={!r}==应用repr()，=={!a}==应用ascii()，=={!s}==应用str()

#### str.format()数字格式化

|    数字    | 格式               | 输出      | 描述                          |
| :--------: | :----------------- | :-------- | ----------------------------- |
| 3.1415926  | {:.2f}             | 3.14      | 保留小数点后两位              |
| 3.1415926  | {:+.2f}            | +3.14     | 带符号保留小数点后两位        |
|     -1     | {:+.2f}            | -1.00     | 带符号保留小数点后两位        |
|  2.71828   | {:.0f}             | 3         | 不带小数点                    |
|     5      | {:0>2d}            | 05        | 数字补零（填充左边，宽度为2） |
|     5      | {:x<4d}            | 5xxx      | 数字补x（填充右边，宽度为4）  |
|     10     | {:x<4d}            | 10xx      | 数字补x（填充右边，宽度为4）  |
|  1000000   | {:,}               | 1,000,000 | 以逗号分隔的数字格式          |
|    0.25    | {:.2%}             | 25.00%    | 百分比格式                    |
| 1000000000 | {:.2e}             | 1.00e+09  | 指数记法                      |
|     13     | {:10d}             | 13        | 右对齐（默认，宽度为10）      |
|     13     | {:<10d}            | 13        | 左对齐（宽度为10）            |
|     13     | {:^10d}            | 13        | 中间对齐（宽度为10）          |
|     11     | ‘{:b}’.format(11)  | 1011      | 进制                          |
|            | ‘{:d}’.format(11)  | 11        |                               |
|            | ‘{:o}’.format(11)  | 13        |                               |
|            | ‘{:x}’.format(11)  | b         |                               |
|            | ‘{:#x}’.format(11) | 0xb       |                               |
|            | ‘{:#X}’.format(11) | 0XB       |                               |

`split()`通过指定分隔符对字符串进行切片，若num有指定值，则分隔num+1个子字符串

```python
str.split(str='', num=string.count(str))
```

`rjust()`返回一个原字符串，右对齐，并使用空格填充至指定长度width的新字符串，若长度小于原字符串长度则返回原字符串

```python
str.rjust(width[, fillchar])

str = 'this is string example....wow!!!'
print(str.rjust(50, '*'))
```

`strip()`方法用于移除字符串头尾指定的字符（默认为空格）或字符串序列（只要是序列中包含的字符即会被去除）

```python
str.strip([chars])
```



## 函数

### 内建函数（Built-in Functions）

Python官网中对各内建函数有详细的[说明文档](<https://docs.python.org/3/library/functions.html>)

`repr()`函数将对象转化为供解释器读取的形式，返回一个对象的string形式

```python
repr(object)
```

`eval()`函数用来执行一个字符串表达式，并返回表达式的值

```python
eval(expression, globals = None, locals = None)

x = 7
eval('3 * x')
eval('pow(2,2)')
```

`ascii()`函数类似`repr()`函数，返回一个表示对象的字符串，但对于字符串中的非ASCⅡ字符则返回通过repr()函数使用\x，\u，\U编码的字符，生成字符串类似Python2中的repr()返回值

```python
ascii(object)
```

`sum()`方法对系列（可迭代对象，如列表、元组、集合）进行求和计算

```python
sum(iterable[, start])
# start为指定相加的参数，默认为0
```



### 创建函数

`def` 和 `return`（可选项）是关键字（keyword）

```python
def function(arg1, arg2):
    return 'something'
```



### 格式化输出

| 格式 |                     描述                     |
| :--: | :------------------------------------------: |
|  %%  |                  百分号标记                  |
|  %c  |                字符及其ASCⅡ码                |
|  %s  |                    字符串                    |
|  %d  |             有符号整数（十进制）             |
|  %u  |             无符号整数（十进制）             |
|  %o  |             无符号整数（八进制）             |
|  %x  |            无符号整数（十六进制）            |
|  %X  |        无符号整数（十六进制大写字符）        |
|  %e  |            浮点数字（科学计数法）            |
|  %E  |       浮点数字（科学计数法，用E代替e）       |
|  %f  |           浮点数字（用小数点符号）           |
|  %g  |      浮点数字（根据值的大小采用%e或%F）      |
|  %G  |             浮点数字（类似于%g）             |
|  %p  |      指针（用十六进制打印值的内存地址）      |
|  %n  | 存储输出字符的数量放进参数列表的下一个变量中 |



### 传递参数

- 位置参数
- 关键词参数
- 默认参数
  - 可选项，函数调用时给出非默认参数即可
  - 在定义参数的时候给参数赋值即可
  
  



## 循环与判断

### 逻辑控制与循环

#### 比较运算（comparision)

- `==` `!=` `>` `<` `<=` `>=` 
- 多条件比较
- 变量间的比较
- 字符的比较（严格区分大小写）
- 函数运行结果间的比较

不同类型的对象不能使用~~“<, >, <=, >=”~~进行比较，却可以使用`==`和`!=`

==浮点==和==整型==虽属不同类型，但不影响比较运算结果

#### 成员运算符与身份运算符（membership&identify operators）

- `in`	测试前者是否存在于`in`后面的集合，表示归属关系的布尔运算符

- `is`&`is not`	身份鉴别的布尔运算符

- 判断对象的布尔值

  ```python
  bool(object)
  ```

#### 布尔运算符（Boolean Operators）

| 运算符  | 描述                                                         |
| ------- | ------------------------------------------------------------ |
| not x   | 如果x是True，则返回Flase，否则返回True                       |
| x and y | 并且。如果x和y都是True，则返回True；如果x和y有一个是Flase，则返回False |
| x or y  | 或者。如果x或y有其中一个是True，则返回True；如果x和y都是False，则返回False |



### 条件控制

```python
if condition:
	do something
else:
	do something
```

多条件判断

```python
if condition:
    do something
elif condition:
    do something
else:
    do something
```



### 循环（for）

`for`循环

```python
for item in iterable:
    do something
```

嵌套循环

```python
for i in range(1, 10):
    for j in range(1, 10):
        print('{} X {} = {}'.format(i, j, i*j))
```

`while`循环

```python
while condition:
    do something
```



## 数据结构

### 列表（list）

1. 每个元素都可变
2. 元素有序，每个元素都有一个位置
3. 可容纳Python中的任何对象

`append()`方法用于在列表末尾添加新的对象

```python
list.append(obj)	#无返回值，但会修改原来的列表
```

`insert()`必须指定在列表中要插入新的元素的位置，实际插入位置在其之前；若插入位置不存在，则会放在列表的最后位置

```python
fruit = ['pineapple', 'pear']
fruit.insert(1, 'grape')
# fruit[0: 0] = ['Orange']
```

`extend()`添加多个元素

```python
fruit.extend('grape')	#拆为单个字符依次插入
fruit.extend(['grape'])	#插入连续字符串‘grape’
fruit.extend(['grape','banana'])	#插入两个字符串['grape','banana']
```

`remove()`删除列表中的元素

```python
fruit.remove('grape')
# del fruit[1]
```

修改替换其中的元素

```python
fruit[0] = 'Grapefruit'
```

`sort()`对列表进行排序，并返回None

```python
fruit.sort()
```

`pop()`函数用于移除列表中的一个元素（默认最后一个元素），并返回该元素的值

```python
list.pop([index = -1])
```

### 字典（Dictionary）

1. 必须以**键值对**的形式出现
2. **键**不可重复，而**值**可以重复
3. **键（key）**不可修改，**值（value）**是可变的，可以是任何对象

**添加**元素

```python
dict['yoku'] = 'youku'
```

`update()`添加多个元素

```python
dict.update({'fb': 'facebook', 'tsla': 'tesla'})
```

`del`删除字典中的元素

```python
del dict['fb']
```

字典使用的是**花括号**，但在索引内容时仍旧使用的是和列表一样的**方括号**，括号中放入的一定是**键**。字典不能做~~**切片**~~

### 元组（Tuple）

==固化版==列表，不可修改，只可查看索引，方法与列表相同

### 集合（Set）

- 无序、不重复的任意对象
- 集合不能切片、索引
- 可做集合运算，可添加、删除

```python
a_set = {1, 2, 3, 4}
a_set.add(5)		#添加
a_set.discard(5)	#删除
```



### 一些数据结构技巧

#### 多重循环

`sorted()`函数按照长短、大小、英文字母的顺序给每个列表中的元素排序（不改变原列表，复制后排序）

通常`sorted()`比`.sort()`更便捷，但如果你不再需要原列表，后者效率更高

```python
num_list = [6, 2, 7, 4, 1, 3, 5]
print(sorted(num_list))
sorted(num_list, reverse = True)	#逆序整理
```

`zip()`函数主要用于同时整理多个列表

```python
for a, b in zip(num, str):
	print(b, 'is', a)
#注意只是整理，并不会对列表进行排序
```

#### 推导式/解析式（List comprehension）

列表解析式

```python
list = [item for item in iterable]
k = [n for n in range(1,10) if n % 2 == 0]
z = [letter.lower() for letter in 'ABCDEFGHIJKLMN']
```

字典推导式

```python
d = {i:i+1 for i in range(4)}
g = {i:j for i, j in zip(range(1,6), 'abcde')}
g = {i:j.upper() for i, j in zip(range(1, 6), 'abcde')}
```

#### 循环列表时获取元素索引

`enumerate()`函数用于将一个可遍历的数据对象（列表、字符串或元组）组合为一个索引序列，同时列出数据和数据下标，一般用在`for`循环当中。

```python
enumerate(sequence, [start = 0])	#start为下标起始位置
letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
for num, letter in enumerate(letters):
    print(letter, 'is', num+1)
```



### 文件读取格式

#### `open()`参数说明

- `‘r’`：默认值，表示从文件读取数据
- `‘w’`：表示要向文件写入数据，并截断以前的内容
- `‘a’`：表示要向文件写入数据，添加到当前内容尾部
- `‘r+’`：表示对文件进行可读写操作（删除以前的所有数据）
- `‘r+a’`：表示对文件可进行读写操作（添加到当前文件的尾部）
- `‘b’`：表示要读写二进制数据

#### 读文件

只有读取到文档结束符`EOF`时才算读到文件最后，Python会认为字节`\x1A(26)`转换成的字符为结束符。

1. 当使用`‘r’`读取二进制文件时，会出现文档读取不全的问题；碰到`‘0x1A’`就视为文件结束；
2. 当使用`‘rb’`按照二进制位读取是，不会将读取的字节转换成字符，从而避免了上述问题。

#### 写文件

`\n`我们称之为换行符，实际上是`0x0A`（在`Windows`平台上会出现以下问题）

- 使用`‘w’`写入时会将其变为两个字符`0x0D`和`0x0A`，也就是说文件实际长度变成了8；当用`‘r’`读取时，又会自动转换为原来的换行符；
- 若换成`‘wb’`二进制方式写入，则会保持一个字符不变，读取时也是原样读取

事实上，`0x0D`便是回车符，`Linux`平台下不会变，只是用`‘0x0A’`表示换行



## 类与可口可乐

```python
class CocaCola:
    formula = ['caffeine', 'sugar', 'water', 'soda']
```

### 类的实例化

左边创建一个变量，右边写一个类的名称，即为类的实例化，而被实例化后的对象，我们称之为实例（instance）

```python
coke_for_you = CocaCola()
coke_for_me = CocaCola()
```

### 类属性的引用（Attribute References）

在类的名字后面输入`.`，IDE就会自动联想出之前定义在类里面的属性，即实现了类属性的引用

**类的属性会被所有类的实例共享**

```python
coke_for_you.formula == coke_for_me.formula
```

类的属性与一般变量并无区别

### 实例属性（Instance Attribute）

```python
coke_for_China = CocaCola()
coke_for_China.local_logo = '可口可乐'
```

类属性与实例属性的引用在**引用方式**上来看完全一致

### 实例方法（Instance Method）

```python
class CocoCola:
    formula = ['caffeine','sugar','water','soda']
    def drink(self):
        print('Energy!')
        
coke = CocaCola
coke.drink() == CocaCola.drink(coke)
```

被实例化的对象会被编译器传入后面方法的括号中，作为第一个参数

#### 更多参数

```python
class CocaCola:
	formula = ['caffeine','sugar','water','soda']
	def drink(self, how_much):
        if how_much == 'a sip':
            print('Cool~')
        elif how_much == 'whole bottle'
        	print('Headache')
ice_coke = CocaCola()
ice_coke.drink('a sip')
```

### “魔术方法”

`__init__()`会在创建实例时自动处理很多事情，例如新增实例属性

```python
class CocoCola:
    formula = ['caffeine','sugar','water','soda']
    def __init__(self):		#self代表着实例对象本身，在所有的方法声明中都存在，并默认作为第一参数传入
        self.local_logo = '可口可乐'
    def drink(self):
        print('Energy!')
coke = CocaCola()
print(coke.local_logo)
```

除了必写的`self`参数之外，`__init()__`也可以有自己的参数，通过在实例化的时候往类后面的括号中放进参数，相应的所有参数都会传递到这个特殊的`__init()__`方法中

```python
class CocaCola:
    formula = ['caffeine','sugar','water','soda']
    def __init__(self, logo_name):
        self.local_logo = logo_name
    def drink(self):
        print('Energy!')
coke = CocaCola('可口可乐')
print(coke.local_logo)
```

### 类的继承（Inheritance）

在新的类`CaffeineFree`后面的括号中加入`CocaCola`即表示这个类是继承于`CocaCola`这个父类的，类中的变量和方法可以完全被子类继承，但如需又特殊的改动可以加以**覆盖（Override）**

```python
class CocaCola:
    calories =  140
    sodium = 45
    total_carb = 39
    caffeine = 34
    ingredients = [
        'High Fructose Corn Syrup',
        'Carbonated Water',
        'Phosphoric Acid',
        'Natural Flavors',
        'Caramel Color',
        'Caffeine'
    ]
    def __init__(self, logo_name):
        self.local_logo = logo_name
    def drink(self):
        print('You got {} cal energy!'.format(self.calories))

class CaffeineFree(CocaCola):
    caffeine = 0
    ingredients = [
        'High Fructose Corn Syrup',
        'Carbonated Water',
        'Phosphoric Acid',
        'Natural Flavors',
        'Caramel Color'
    ]

coke_a = CaffeineFree('CocaCola-FREE')
coke_a.drink()
print(coke_a.local_logo)
```

- **类的属性**被重新赋值后，相应的类属性引用也**会被改写**
- **实例属性**被重新赋值后，相应的类属性引用**不会被改写**
- **类属性**与**实例属性**名称相同时，`.`后的引用与**实例属性**一致

`__dict__`是一个类的特殊属性，是一个字典，用于储存**类**或者**实例的属性**

#### 属性的引用机制

自外而内，当创建一个实例后，准备开始引用属性，这时候编译器会先搜索该实例是否拥有该属性，如果有，则引用；如果没有，将搜索这个实例所属的类是否有这个属性，如果有，则引用，没有就报错



## 生成器

生成器与迭代器都可以迭代，但只能读取一次，因为它并不是将所有值放在内存里，而是**实时生成**的数据

```python 
mylist = [x*x for x in range(3)]	#[]迭代器
mygenerator = (x*x for x in range(3))	#()生成器
```

### yield关键字

类似于`return`，但返回值是一个**生成器**，当调用这个函数时，函数内部的代码不会立即执行，只有当你使用`for`循环进行迭代时才会执行。

```python
def createGenerator():
    mylist = range(3)
    for i in mylist:
        yield i*i

mygenerator = createGenerator()
print(mygenerator)
for i in mygenerator:
    print(i)
```

## 使用第三方库

[Awesome Python](<https://awesome-python.com/>)上收录了比较全面的第三方库

### 安装第三方库

#### 在终端/命令行中安装

- 安装pip

  检查pip版本

  ```python 
  pip --version
  ```

  若未显示版本则需要先安装对应系统的pip

- 使用pip安装库

  ```python 
  pip install PackageName		#安装到python2中
  pip3 install PackageName	#安装到python3中
  ```

  若python2和3都有安装，则可能会遇到安装目录的问题，可换成：

  ```python
  python -m pip install PackageName	#安装到python2中
  python3 -m pip install PackageName	#安装到python3中
  ```

几个常用的pip指令

```python
pip install --upgrade pip	#升级pip
pip uninstall flask			#卸载库
pip list					#查看已安装库
```

#### 手动安装

- 进入`pypi.python.org`，搜索想要安装的库的名字

  - `.exe`文件——最为方便，一路`next`安装即可
  - `.whl`文件——可自动安装依赖包
  - `.zip/tar.zip/tar.bz2`压缩包——源码，要求用户已经安装了这个包所依赖的其他包。例如，`pandas`依赖于`numpy`

- `.whl`类文件的安装方法

  - 命令行输入：

  ```python
  pip3 install wheel	#python2换为pip，不能报错
  ```

  - 在资源管理器中确认你下载的`.whl`类文件路径，并输入命令：

  ```python
  cd path			#path即为文件路径
  pip3 install xxx.whl
  ```

- `.zip/tar.zip/tar.bz2`压缩包的安装方法

  - 解压包，确认`setup.py`文件，并确认下载路径`path`

  ```python
  cd path
  ```

  - 安装包

  ```python
  python3 setup.py install
  ```

  `Windows`用户如果想卸载库，进入对应python版本的路径，找到`site-packages`文件夹，直接删除对应库的文件即可



## 学习资源

1. [图灵社区](<http://www.ituring.com.cn/>)

   技术方面的中文书籍。有诸多免费或付费电子书，最新的翻译书籍也能找到

2. [Python-100天从新手到大师](<https://github.com/jackfrued/Python-100-Days>)



## 模块

### random

`random`是内建（build-in）函数，作用是产生随机数

#### 导入模块

```python
import random
>>> dir(random)	# 查看random模块下有哪些函数
```

#### 常用函数

```python
random.randint(1,10)	# 产生1~10（包括1和10）的一个随机数（int型）
random.random()			# 产生0~1（包括0不包括1）的一个随机浮点数
random.uniform(a,b)		# 生成a~b的随机浮点数，不必考虑上下限大小关系
random.choice(seq)		# 从序列（列表/元组/字符串）中随机选取一个元素
random.randrange(start,stop,step)	# 生成一个从start到stop（不包括stop）间隔为step的随机整数
random.sample(p,k)		# 从p序列中随机获取k个元素，生成一个新序列，且不改变原序列，支持多种专业随							 机算法
random.shuffle(x)		# 把序列x中的元素顺序打乱，将直接改变原序列，无返回值
```

### math

`math`模块是Python标准库中的，所以可以直接使用

```python
import math
>>> dir(math)	# 查看模块中所含的工具
>>> help(math)	# 展示math模块中函数的使用方法
```

#### 常用函数

```python
math.e				# 自然常数e
math.pi				# 圆周率pi
math.degrees(x)		# 弧度转度
math.radians(x)		# 度转弧度
math.exp(x)			# 返回e的x次方
math.expm1(x)		# 返回（e的x次方）-1
math.log(x,[,base])	# 返回x的以base为底的对数，base默认为e
math.log10(x)		# 返回x的以10为底的对数
math.log1p(x)		# 返回1+x的自然对数
math.pow(x,y)		# 返回x的y次方
math.sqrt(x)		# 返回x的平方根
math.ceil(x)		# 返回不小于x的整数
math.floor(x)		# 返回不大于x的整数
math.trunc(x)		# 返回x的整数部分
math.modf(x)		# 返回x的小数和整数
math.fabs(x)		# 返回x的绝对值
math.fmod(x,y)		# 返回x%y（取余）
math.fsum([x,y,...])# 返回无损精度的和
math.factorial(x)	# 返回x的阶乘
math.isinf(x)		# 若x为无穷大，返回True；否则，返回False
math.isnan(x)		# 若x不是数字，返回True；否则，返回Flase
math.hypot(x,y)		# 返回以x和y为直角边的斜边长
math.copysign(x,y)	# 若y<0，返回-1乘以x的绝对值；否则，返回x的返回值
math.frexp(x)		# ldexp的反函数，返回一个二元组
math.ldexp(m,i)		# 返回m乘以（2的i次方）
math.sin(x)	
math.asin(x)
math.cos(x)
math.acos(x)
math.tan(x)
math.atan(x)
math.atan2(x,y)
math.sinh(x)		# 返回x的双曲正弦函数
math.asinh(x)		# 返回x的反双曲正弦函数
math.cosh(x)		# 返回x的双曲余弦函数
math.acosh(x)		# 返回x的反双曲余弦函数
math.tanh(x)		# 返回x的双曲正切函数
math.atanh(x)		# 返回x的反双曲正切函数
math.erf(x)			# 返回x的误差函数
math.erfc(x)		# 返回x的余误差函数
math.gamma(x)		# 返回x的伽马函数
math.lgamma(x)		# 返回x的绝对值的自然对数的伽马函数
```

### os

```python
os.mkdir(path,mode)	#以数字权限模式创建目录，默认0777（十八进制）
```

#### os.path

主要用于获取文件属性

```python
os.path.exists(path)	#如果路径path存在，返回true；否则返回false
```



 https://docs.python-guide.org/writing/structure/ 

# Structuring Your Project

[Sample Repository](https://github.com/navdeep-G/samplemod)

```
README.rst
LICENSE
setup.py
requirements.txt
sample/__init__.py
sample/core.py
sample/helpers.py
docs/conf.py
docs/index.rst
tests/test_basic.py
tests/test_advanced.py
```

## Sample[`./sample/` or `./sample.py`]

**核心代码**存放处

## License[`./LICENSE`]

如果不确定项目适合用哪一种**执照许可**，可参考[choosealicense.com](https://choosealicense.com/)

不适用任何license发布代码当然也是可以的，只是可能会对一些想要使用代码的人造成潜在困扰

## Setup.py[`./setup.py`]

**对所需调用的包进行集中管理**

[setup.py样例](https://github.com/navdeep-G/setup.py)

## Requirements File[`./requirements.txt`]

开发**依赖项**

此依赖项安装可并入Setup.py文件

具体写法格式见[Example Requirements File](https://pip.pypa.io/en/stable/reference/pip_install/#requirements-file-format)

## Documentation[`./docs/`]

软件包的相关参考文档

别的文件路径下不应该有类似的文档

## Test Suite[`./test_sample.py` or `./tests`]

软件包整体与各功能单元的测试

各个测试包必定需要调用不同的软件包，也包括很多额外的调试工具包，所以建议单独写一个文件`tests/context.py`，例如：

```python
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import sample
```

之后在同级目录下的测试包中引用：

```python
from .context import sample
```

## Makefile[`./Makefile`]

定义通用任务

`Sample Makefile`

```python
init:
    pip install -r requirements.txt

test:
    py.test tests

.PHONY: init test
```



# Structure of Code

> - 大量的循环依赖