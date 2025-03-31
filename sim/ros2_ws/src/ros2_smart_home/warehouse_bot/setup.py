from setuptools import setup
import os
from glob import glob

package_name = "warehouse_bot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/warehouse_bot"]),
        ("share/" + package_name, ["package.xml"]),
        # launch 파일 자동 포함
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # config 파일 포함
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SSAFY12thE102",
    maintainer_email="j1113019@naver.com",
    description="Warehouse autonomous robot core nodes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_launch = warehouse_bot.main_launch:main",
            "run_mapping = warehouse_bot.slam.run_mapping:main",
            "auto_mapping = warehouse_bot.slam.auto_mapping_frontier:main",
            "odom = warehouse_bot.slam.odom:main",
        ],
    },
)
