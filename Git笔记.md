# Git笔记

[廖雪峰的官方网站](https://www.liaoxuefeng.com/wiki/896043488029600)

## 安装Git

### 在Linux上安装

Debian或Ubuntu Linux版本

```shell
$ sudo apt-get install git
```

其他Linux版本，官网下载源码安装

```shell
$ ./config
$ make
$ sudo make install
```

### 在Windows上安装

在[Git官网](https://git-scm.com/downloads)上直接下载安装程序，默认选项安装即可

安装完成后在开始菜单中找到**Git** -> **Git Bash**

```shell
$ git config --global user.name "jeff"
$ git config --global user.email "jeffery-areo@outlook.com"
```

`git config`命令中的`--global`参数表示这台机器上的所有Git仓库都将会使用这个配置



## 创建版本库(Repository)

合适的路径（Windows下确保路径无中文）下创建空目录

```shell
$ mkdir laptopgit
$ cd laptopgit
$ pwd
/home/jeff/laptopgit
```

将该目录收入**Git**可管理的仓库

```shell
$ git init
Initialized empty Git repository in /home/jeff/laptopgit/.git/
```

当前目录下会生成一个`.git`目录，用以跟踪管理版本库

### 将文件添加至版本库

> **注**：所有版本控制系统都只能跟踪**文本文件**（`.txt` `.html`以及所有程序代码）的改动，无法跟踪**二进制文件**

> **Windows用户注**：不要用**记事本**编辑任何文本文件，`utf-8`编码机制会自动在每个文件开头加入`0xefbbbf`的十六进制字符

使用命令`git add`告诉**Git**，将文件添加到仓库（没有消息就是好消息）

```shell
$ git add Readme.md		#可能需要sudo
```

使用命令`git commit`告诉**Git**，将文件提交到仓库

````shell
$ git commit -m "worte a readme file"
[master (root-commit) 2af4647] worte a readme file
 1 file changed, 0 insertions(+), 0 deletions(-)
 create mode 100644 Readme.md
````

`-m`后输入本次提交内容的文字说明

`commit`可一次性提交多个文件

```shell
$ git add file1.txt
$ git add file2.txt file3.txt
$ git commit -m "add 3 files."
```



## 时光穿梭机

尝试修改已经提交的文本文件内容，运行`git status`查看当前状态

```shell
$ git status
On branch master
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git checkout -- <file>..." to discard changes in working directory)

        modified:   Readme.md                                                                                           
no changes added to commit (use "git add" and/or "git commit -a")
```

上述命令输出告诉我们`Readme.md`被修改了，但还没有提交

`git diff`可进一步查看修改内容

```shell
$ git diff Readme.md		#可能需要sudo
diff --git a/Readme.md b/Readme.md
index 67a02bf..98f6038 100644                                                                                           --- a/Readme.md                                                                                                         +++ b/Readme.md                                                                                                         @@ -1,4 +1,4 @@ 
-Git is a version control system.                                                                                       -Git is free.                                                                                                           +Git is a distributed version control system.^M                                                                         +Git is a free software.^M  
```

### 版本回退

版本提交历史可通过命令`git log`查看

```shell
$ git log
$ git log --pretty=online	#只显示commit id，即版本号
```

将当前版本回退到上一个版本`git reset`

```shell
$ git reset --hard HEAD^		#HEAD表示当前版本，HEAD^表示上一个版本，上上个版本为HEAD^^,上100个版本为HEAD~100
```

> **注**：如果想再返回到回退操作前的版本则需要记住该版本对应的**commit id**，对应我的append GPL版本号为46e7aae01f2b00fae639b77a9e9326c57708fc0f，没必要写全，Git能够靠前几位找到

```shell
$ git reset --hard 46e7aae01f2b00fae639b77a9e9326c57708fc0f
HEAD is now at 46e7aae append GPL
```

如果忘了回退版本的版本号`commit id`，可以通过`git reflog`查看命令历史

```shell
$ git reflog
$ git reset --hard 46e7aae
```

### 管理修改

`Git`跟踪并管理的是**修改**而不是**文件**，每次修改如果不用`git add`到暂存区，则不会加入到`commit`中

### 撤销修改

`git checkout -- <file>`可丢弃对应文件在工作区的全部修改

```shell
$ git checkout -- Readme.md
```

`git reset HEAD <file>`可以把暂存区的修改撤销掉（unstage），重新放回工作区

```shell
$ git reset HEAD Readme.md
```

### 删除文件

在工作区删除文件后，确认要从版本库删除该文件使用

```shell
$ git rm file3.txt
$ git commit -m "remove file3.txt"
```

工作区错删后恢复到与版本库内一致

```shell
$ git checkout -- file3.txt 
```

> **注**：从来没有被提交到版本库内的文件误删后是无法恢复的



## 远程仓库

由于本地`Git`仓库与`Github`仓库之间的传输是通过`SSH`加密的，使用前需要一点设置

创建`SSH Key`

```shell
$ ssh-keygen -t rsa -C "jeffery-areo@outlook.com"
```

 一路回车，使用默认值即可，由于这个`Key`也不是用于军事目的，所以也无需设置密码 

可以在用户主目录里找到`.ssh`目录，里面有`id_rsa`和`id_rsa.pub`两个文件，这两个就是SSH Key的秘钥对，`id_rsa`是私钥，不能泄露出去，`id_rsa.pub`是公钥，可以放心地告诉任何人 

登录`Github`–>打开`Account settings` –>` SSH Keys `–>` Add SSH Key `–>粘贴`id_rsa.pub`文件的内容 

### 添加远程库

 登陆GitHub –> ` Create a new repo ` –> ` Create repository ` 

 我们根据GitHub的提示，在本地的`learngit`仓库下运行命令： 

```shell
$ git remote add origin git@github.com:Jeffery-Chen/laptopgit.git
$ git remote add origin https://github.com/Jeffery-Chen/laptopgit.git
```

 添加后，远程库的名字就是`origin`，这是Git默认的叫法，也可以改成别的，但是`origin`这个名字一看就知道是远程库 

 把本地库的所有内容推送到远程库上： 

```shell
$ git push -u origin master
```

 由于远程库是空的，我们第一次推送`master`分支时，加上了`-u`参数，Git不但会把本地的`master`分支内容推送的远程新的`master`分支，还会把本地的`master`分支和远程的`master`分支关联起来，在以后的推送或者拉取时就可以简化命令 

从现在起，只要本地作了提交，就可以通过命令：

```shell
$ git push origin master
```

### 从远程库克隆

克隆一个远程库到本地（`ssh`或`https`等协议均可）

```shell
$ git clone https://github.com/Jeffery-Chen/laptopgit.git
$ git clone git@githubcom:Jeffery-Chen/laptopgit.git
```

使用`https`除了速度慢以外，还有个最大的麻烦是每次推送都必须输入口令，但是在某些只开放http端口的公司内部就无法使用`ssh`协议而只能用`https`

### 删除远程库

```shell
$ git remote rm origin
```

### 多个远程库关联

关联时的库名称不再使用`origin`，而是按照需求起好各自的名字，例如`github` `gitee`等

## 分支管理

### 创建与合并分支

创建`dev`分支，然后切换到`dev`分支 

```shell
$ git checkout -b dev		#-b表示创建并切换
Switched to a new branch 'dev'
```

相当于以下两条命令： 

```shell
$ git branch dev
$ git checkout dev
Switched to branch 'dev'
```

用`git branch`命令查看当前分支（带 * 标记） 

```shell
$ git branch
* dev
  master
```

在`dev`分支上正常提交 

```shell
$ git add Readme.md
$ git commit -m "branch test"
```

切换回`master`分支： 

```shell
$ git checkout master
```

把`dev`分支的工作成果合并到`master`分支上： 

```shell
$ git merge dev		#合并指定分支到当前分支
```

合并完成后，就可以放心地删除`dev`分支 

```shell
$ git branch -d dev
Deleted branch dev (was 3b32095).
$ git branch
* master
```

切换分支这个动作，用`switch`更科学 

```shell
$ git switch -c dev		#创建并切换到新的dev分支
$ git switch master		#直接切换到已有的master分支
```

### 解决冲突

如果不同分支的提交内容有冲突则无法“快速合并”，自动合并后需手动解决冲突内容

使用`git log --graph`命令可以查看分支的合并图

```shell
$ git log --graph --pretty=oneline --abbrev-commit
```

### 分支管理策略

通常，合并分支时，如果可能，Git会用`Fast forward`模式，但这种模式下，删除分支后，会丢掉分支信息

```shell
$ git merge --no-ff -m "merge with no-ff" dev		#--no-ff参数表示禁用Fast forward模式，生成新的commit
```

### Bug分支

`stash`功能用以将当前工作现场“储藏”起来

```shell
$ git stash		#截取当前工作现场
#完成其他分支各项工作后回到原分支下
$ git stash	list#查看保存的工作现场
$ git stash apply stash@{0}		#恢复指定stash
$ git stash drop	#删除stash内容
$ git stash pop		#恢复并删除stash
```

`cherry-pick`命令用以复制一个特定的提交到当前分支

```shell
$ git branch
* dev
  master
$ git cherry-pick 4c805e2	#4c805e2即为想要复制的特定提交id
```

### Feature分支

强行删除一个没有合并过的分支

```shell
$ git branch -D <name>
```

### 多人协作

查看远程库信息（克隆到本地的`master`分支与远程的`master`分支对应，远程仓库默认名称为origin）

```shell
$ git remote
$ git remote -v		#更详细的信息
```

推送分支

```shell
$ git push origin master
$ git push origin dev
```

抓取分支（以dev为例）

```shell
$ git branch --set-upstream-to=origin/dev dev	#设置本地dev和远程origin/dev链接
$ git pull		#确保本地更新，如果没有上一步的链接，则会提示no tracking information
#可能会出现合并冲突，手动解决后提交并push
$ git push origin dev	#将自己的dev推送到远程
```

### Rebase

```shell
$ git rebase
```

 `rebase`操作可以把本地未push的分叉提交历史整理成直线 



## 标签管理

`Git`的标签实质上就是指向某个`commit`的指针

创建标签

```shell
$ git tag -a v0.1 -m "version 0.1 released" 1094adb
```

查看标签

```shell
$ git tag
$ git show <tagname>
```

 默认标签是打在最新提交的commit上的

为历史提交打标签

```shell
$ git log --pretty=oneline --abbrev-commit		#找到历史提交的commit id
12a631b (HEAD -> master, tag: v1.0, origin/master) merged bug fix 101
4c805e2 fix bug 101
e1e9c68 merge with no-ff
f52c633 add merge
cf810e4 conflict fixed
5dc6824 & simple
14096d0 AND simple
b17d20e branch test
d46f35e remove test.txt
b84166e add test.txt
519219b git tracks changes
e43a48b understand how stage works
1094adb append GPL
e475afc add distributed
eaadf4e wrote a readme file
$ git tag v0.9 f52c633
```

> **注**：标签总是与某个commit挂钩

删除标签（本地）

```shell
$ git tag -d <tagname>
```

推送标签至远程

```shell
$ git push origin <tagname>
$ git push origin --tags	#一次性推送全部本地标签
```

删除远程标签

```shell
$ git tag -d <tagname>
$ git push origin :refs/tags/<tagname>
```



## 自定义Git

显示颜色

```
$ git config --global color.ui true
```

忽略特殊文件

```shell
#在Git工作区的根目录下创建一个特殊的.gitignore文件，然后把要忽略的文件名填进去，Git就会自动忽略这些文件
$ git check-ignore -v <filename>	#检查被忽略的文件遵从的规则
$ git add -f <filename>		#强制添加被忽略的文件
```

不需要从头写.gitignore文件，GitHub已经为我们准备了[各种配置](https://github.com/github/gitignore)文件，只需要组合一下就可以使用了



## 配置别名

```shell
$ git config --global alias.st status	#用st表示status
$ git config --global alias.unstage 'reset HEAD'
$ git config --global alias.lg "log --color --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit"	#美观是一辈子的事
```

配置文件

```shell
$ cat .git/config 		#查看当前仓库内的配置文件
[core]
    repositoryformatversion = 0
    filemode = true
    bare = false
    logallrefupdates = true
    ignorecase = true
    precomposeunicode = true
[remote "origin"]
    url = git@github.com:michaelliao/learngit.git
    fetch = +refs/heads/*:refs/remotes/origin/*
[branch "master"]
    remote = origin
    merge = refs/heads/master
[alias]
    last = log -1		#别名就在[alias]后面，要删除别名，直接把对应的行删掉即可
$ cat .gitconfig		#查看当前用户的配置文件（隐藏文件）
[alias]
    co = checkout
    ci = commit
    br = branch
    st = status
[user]
    name = Your Name
    email = your@email.com
```



## 搭建Git服务器

搭建Git服务器需要准备一台运行Linux的机器，强烈推荐用`Ubuntu`或`Debian`，这样，通过几条简单的`apt`命令就可以完成安装，步骤详见[廖雪峰的官方网站](https://www.liaoxuefeng.com/wiki/896043488029600/899998870925664)

