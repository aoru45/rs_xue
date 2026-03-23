from __future__ import annotations

import os
import pathlib
import subprocess
import sys

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext


ROOT = pathlib.Path(__file__).resolve().parent
VCPKG_JSON = ROOT / "vcpkg.json"
VCPKG_INSTALLED = ROOT / "vcpkg_installed"


class CMakeExtension(Extension):
    def __init__(self, name: str) -> None:
        super().__init__(name, sources=[])


class CMakeBuild(build_ext):
    def build_extension(self, ext: Extension) -> None:
        ext_fullpath = pathlib.Path(self.get_ext_fullpath(ext.name)).resolve()
        extdir = ext_fullpath.parent
        build_temp = pathlib.Path(self.build_temp) / ext.name
        build_temp.mkdir(parents=True, exist_ok=True)

        try:
            import pybind11
        except ImportError as exc:
            raise RuntimeError(
                "pybind11 is required to build rs_xue. Install it in the current Python environment first."
            ) from exc

        cfg = "Debug" if self.debug else "Release"
        cmake_args = [
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY={build_temp}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}",
            f"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-Dpybind11_DIR={pybind11.get_cmake_dir()}",
        ]

        cmake_prefix_paths: list[str] = []
        toolchain_file = os.environ.get("CMAKE_TOOLCHAIN_FILE")
        if not toolchain_file:
            vcpkg_root = os.environ.get("VCPKG_ROOT")
            if vcpkg_root:
                candidate = pathlib.Path(vcpkg_root) / "scripts" / "buildsystems" / "vcpkg.cmake"
                if candidate.exists():
                    toolchain_file = str(candidate)

        if toolchain_file:
            cmake_args.append(f"-DCMAKE_TOOLCHAIN_FILE={toolchain_file}")
            if VCPKG_JSON.exists():
                cmake_args.append(f"-DVCPKG_MANIFEST_DIR={ROOT}")

        triplet = os.environ.get("VCPKG_TARGET_TRIPLET")
        if triplet:
            cmake_args.append(f"-DVCPKG_TARGET_TRIPLET={triplet}")

        if VCPKG_INSTALLED.is_dir():
            if not triplet:
                triplet_dirs = sorted(
                    p for p in VCPKG_INSTALLED.iterdir() if p.is_dir() and p.name != "vcpkg"
                )
                if len(triplet_dirs) == 1:
                    triplet = triplet_dirs[0].name
                    cmake_args.append(f"-DVCPKG_TARGET_TRIPLET={triplet}")
            if triplet:
                triplet_root = VCPKG_INSTALLED / triplet
                if triplet_root.is_dir():
                    cmake_prefix_paths.append(str(triplet_root))

        existing_prefix_path = os.environ.get("CMAKE_PREFIX_PATH")
        if existing_prefix_path:
            cmake_prefix_paths.extend(p for p in existing_prefix_path.split(os.pathsep) if p)

        if cmake_prefix_paths:
            deduped_prefixes = list(dict.fromkeys(cmake_prefix_paths))
            cmake_args.append(f"-DCMAKE_PREFIX_PATH={os.pathsep.join(deduped_prefixes)}")

        build_args = ["--config", cfg]
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            build_args.extend(["--parallel", str(os.cpu_count() or 1)])

        subprocess.run(
            ["cmake", "-S", str(ROOT), "-B", str(build_temp), *cmake_args],
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(build_temp), *build_args],
            check=True,
        )


setup(
    name="rs_xue",
    version="1.0.0",
    description="RoboSense LiDAR Python interface",
    packages=find_packages(include=["rs_xue", "rs_xue.*"]),
    ext_modules=[CMakeExtension("rs_xue._rs_xue")],
    cmdclass={"build_ext": CMakeBuild},
    include_package_data=True,
    install_requires=["numpy"],
    python_requires=">=3.8",
    zip_safe=False,
)
