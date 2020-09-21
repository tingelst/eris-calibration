# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from skbuild import setup


def main():
    setup(
        name="eris",
        version="0.0.0",
        description="Hand-eye calibration",
        long_description=open("README.md").read(),
        long_description_content_type="text/markdown",
        url="",
        author="Lars Tingelstad",
        author_email="Lars Tingelstad@ntnu.no",
        license="Apache License, Version 2.0",
        packages=["eris", "_eris"],
        package_dir={"": "modules"},
        cmake_args=[
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INSTALL_RPATH=$ORIGIN",
            "-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=ON",
            "-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=OFF",
        ],
        classifiers=[
            "License :: OSI Approved :: OSI Approved :: Apache Software License",
            "Programming Language :: Python :: 3",
            "Programming Language :: Python :: Implementation :: CPython",
            "Topic :: Scientific/Engineering",
        ],
    )


if __name__ == "__main__":
    main()
