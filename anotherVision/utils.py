import os.path as path
def getWeightPath(fileName:str) -> str:
    """获取pt文件的绝对路径

    Args:
        fileName (str): 文件名(不带.pt)

    Returns:
        str: 对应的绝对路径
    """
    return path.abspath(path.join(path.dirname(__file__), "weights", fileName+'.pt'))

if __name__ == "__main__":
    print(getWeightPath("hello"))