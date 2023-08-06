import os
import json
import random


def split_data(root: str, val_rate: float = 0.2):
    assert os.path.exists(root), "dataset root: {} does not exist.".format(root)

    # 遍历文件夹，一个文件夹对应一个类别
    number_class_list: list[str] = [number_class for number_class in os.listdir(root) if
                                    os.path.isdir(os.path.join(root, number_class))]
    # 排序，保证顺序一致
    number_class_list.sort()

    # 生成类别名称以及对应的数字索引, key是字符型数字， val是数字
    class_indices = dict((key, val) for val, key in enumerate(number_class_list))

    json_str = json.dumps(dict((val, key) for key, val in class_indices.items()), indent=4)
    with open('class_indices.json', 'w') as json_file:
        json_file.write(json_str)

    train_images_path_list = []  # 存储训练集的所有图片路径
    train_images_label_list = []  # 存储训练集图片对应索引信息
    val_images_path_list = []  # 存储验证集的所有图片路径
    val_images_label_list = []  # 存储验证集图片对应索引信息
    every_class_num_list = []  # 存储每个类别的样本总数
    supported_list = [".jpg", ".JPG", ".png", ".PNG"]  # 支持的文件后缀类型

    # 遍历每个文件夹下的文件
    for number_class in number_class_list:
        class_path = os.path.join(root, number_class)
        # 遍历获取supported支持的所有文件路径
        image_paths = [os.path.join(root, number_class, i) for i in os.listdir(class_path)
                       if os.path.splitext(i)[-1] in supported_list]
        # 获取该类别对应的索引
        image_class = class_indices[number_class]
        # 记录该类别的样本数量
        every_class_num_list.append(len(image_paths))
        # 按比例随机采样验证样本
        val_path = random.sample(image_paths, k=int(len(image_paths) * val_rate))

        for img_path in image_paths:
            if img_path in val_path:  # 如果该路径在采样的验证集样本中则存入验证集
                val_images_path_list.append(img_path)
                val_images_label_list.append(image_class)
            else:  # 否则存入训练集
                train_images_path_list.append(img_path)
                # label的list里装着int型数字
                train_images_label_list.append(image_class)

    print("{} images were found in the dataset.".format(sum(every_class_num_list)))
    print("{} images for training.".format(len(train_images_path_list)))
    print("{} images for validation.".format(len(val_images_path_list)))

    return train_images_path_list, train_images_label_list, val_images_path_list, val_images_label_list
