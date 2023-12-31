import os
import json
import random


def split_data(root: str, test_rate: float = 0.2):
    random.seed(0)  # 保证随机结果可复现
    assert os.path.exists(root), "dataset root: {} does not exist.".format(root)

    # 遍历文件夹，一个文件夹对应一个类别
    number_class_list: list[str] = [number_class for number_class in os.listdir(root) if
                                    os.path.isdir(os.path.join(root, number_class))]
    # 排序，保证顺序一致
    number_class_list.sort()

    # 生成类别名称以及对应的数字索引, key是字符型数字， index是数字
    class_indices = dict((name_class, index) for index, name_class in enumerate(number_class_list))

    json_str = json.dumps(dict((index, name_class) for name_class, index in class_indices.items()), indent=4)
    with open('class_indices.json', 'w') as json_file:
        json_file.write(json_str)

    train_images_path_list = []  # 存储训练集的所有图片路径
    train_images_label_list = []  # 存储训练集图片对应索引信息
    test_images_path_list = []  # 存储验证集的所有图片路径
    test_images_label_list = []  # 存储验证集图片对应索引信息
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
        test_path = random.sample(image_paths, k=int(len(image_paths) * test_rate))

        for img_path in image_paths:
            if img_path in test_path:  # 如果该路径在采样的验证集样本中则存入验证集
                test_images_path_list.append(img_path)
                test_images_label_list.append(image_class)
            else:  # 否则存入训练集
                train_images_path_list.append(img_path)
                # label的list里装着int型数字
                train_images_label_list.append(image_class)

    print("{} images were found in the dataset.".format(sum(every_class_num_list)))
    print("{} images for training.".format(len(train_images_path_list)))
    print("{} images for testing.".format(len(test_images_path_list)))

    images_path_list_dir = {
        "train": train_images_path_list,
        "test": test_images_path_list
    }

    images_label_list_dir = {
        "train": train_images_label_list,
        "test": test_images_label_list
    }

    return images_path_list_dir, images_label_list_dir
