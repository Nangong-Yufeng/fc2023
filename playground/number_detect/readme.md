文件dataset分为shuzi文件和main.py、my_dataset.py、RGB2GRAY.py、utils.py、class_indices.json

shuzi:分为0-9这10个文件，对应的数字存到对应的文件夹中，这些数字来自拍摄的两位数字组成的数据集，首先要运行RGB2GRAY.py将图片分为两部分，并且灰度化，然后再存入对应的文件夹。

my_dataset.py:主要实现了数据集的导入，并且只能是灰度图(若需要RGB可以改参数)。

main.py:
	main函数：根据代码自动判断使用GPU还是CPU
		   创建好了train_loader,test_loader可照着train_loader书写。
	Net函数:降为10维度。
	criterion:创建交叉熵损失。
	optimizer:创建SGD优化器。
	train：进行梯度下降算法计算，每300次打印 一次。
	test：不进行梯度下降算法，若预测结果和标签一致，则correct++，最终输出正确率和预测的结果(预测的结果还没改）。
	if __name__=="__main__":开始训练，epoch设置为100。