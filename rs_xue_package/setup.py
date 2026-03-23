from setuptools import setup, find_packages
import os
import glob
import shutil

# 当前包目录
here = os.path.abspath(os.path.dirname(__file__))
pkg_dir = os.path.join(here, 'rs_xue')
project_root = os.path.abspath(os.path.join(here, '..'))

# Ensure `ext_files` is always defined and only populated when project_root exists.
ext_files = []
if os.path.isdir(project_root):
    ext_files = glob.glob(os.path.join(project_root, '**', 'rs_xue*.so'), recursive=True) + \
                glob.glob(os.path.join(project_root, '**', 'rs_xue*.pyd'), recursive=True)

if os.path.isdir(pkg_dir) and ext_files:
    for f in ext_files:
        try:
            shutil.copy2(f, pkg_dir)
            print(f"Copied binary extension: {f} -> {pkg_dir}")
        except Exception as e:
            print(f"Warning: failed to copy {f} -> {pkg_dir}: {e}")

# 复制运行时依赖 DLL（PcapPlusPlus/zlib 以及可选 Npcap），以避免导入时找不到模块
copied_dlls = []
if os.path.isdir(pkg_dir):
    dll_candidates = []
    # 1) 与扩展同目录的 DLL
    for f in ext_files:
        d = os.path.dirname(f)
        dll_candidates += glob.glob(os.path.join(d, '*.dll'))
    # 2) vcpkg 安装目录中的常见依赖 DLL
    vcpkg_root = os.environ.get('VCPKG_ROOT')
    if vcpkg_root:
        vcpkg_bin = os.path.join(vcpkg_root, 'installed', 'x64-windows', 'bin')
        for name in ('Pcap++.dll', 'Packet++.dll', 'Common++.dll', 'zlib1.dll'):
            p = os.path.join(vcpkg_bin, name)
            if os.path.exists(p):
                dll_candidates.append(p)
    # 3) 可选：系统中的 Npcap（仅本地安装时帮助，分发时建议用户单独安装 Npcap）
    npcap_paths = [
        r"C:\\Windows\\System32\\Npcap\\wpcap.dll",
        r"C:\\Windows\\System32\\Npcap\\Packet.dll",
        r"C:\\Windows\\System32\\wpcap.dll",
        r"C:\\Windows\\System32\\Packet.dll",
        os.path.join(os.environ.get('ProgramFiles', r"C:\\Program Files"), 'Npcap', 'wpcap.dll'),
        os.path.join(os.environ.get('ProgramFiles', r"C:\\Program Files"), 'Npcap', 'Packet.dll'),
    ]
    for p in npcap_paths:
        if os.path.exists(p):
            dll_candidates.append(p)

    # 去重并复制
    seen = set()
    for p in dll_candidates:
        key = os.path.basename(p).lower()
        if key in seen:
            continue
        seen.add(key)
        try:
            shutil.copy2(p, pkg_dir)
            copied_dlls.append(p)
            print(f"Copied runtime DLL: {p} -> {pkg_dir}")
        except Exception as e:
            print(f"Warning: failed to copy DLL {p} -> {pkg_dir}: {e}")

setup(
    name='rs_xue',
    version='1.0.0',
    description='RealSense LiDAR Python Interface By Xue',
    author='Aoru.Xue',
    author_email='aoru45@shanghaitech.edu.cn',
    packages=find_packages(),
    package_data={
        'rs_xue': ['*.so', '*.pyd', '*.dll'],
    },
    include_package_data=True,
    python_requires='>=3.8',
    install_requires=[
        'numpy',
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
    ],
    zip_safe=False,
)