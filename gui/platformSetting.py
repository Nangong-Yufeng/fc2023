import platform

def setting():
    sys_platform = platform.platform().lower()
    if "windows" in sys_platform:
        pass
    elif "linux" in sys_platform:
        import os
        os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = '--no-sandbox'
    elif "macos" in sys_platform:
        import sys
        print("我超！果！")
        sys.exit(0)
    else:
        import sys
        print('凡事多想想自己的问题')
        sys.exit(-1)


if __name__ == "__main__":
    setting()