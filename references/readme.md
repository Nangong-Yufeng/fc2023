# References

## 简介

一些可能与飞控、视觉有关的程序

## git submodule 使用方法

1. git clone [fc2023](..) 这个大项目

   ```shell
   git clone https://github.com/Nangong-Yufeng/fc2023.git
   ```

2. 进入fc2023目录

   ```
   cd fc2023
   ```

3. 运行两个命令：`git submodule init`来初始化你的本地配置文件，`git submodule update`来从那个项目拉取所有数据并检出你上层项目里所列的合适的提交

   ```shell
   git submodule init
   ```

   ```shell
   git submodule update
   ```

4. 如果后续submodule的项目有更新，直接在submodule项目下`git pull origin main`即可。

## 参考资料

1. https://blog.csdn.net/Real_Myth/article/details/86286902