from setuptools import setup
from glob import glob
package_name = "malkia_mission"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"]),
                ("share/" + package_name + "/launch", glob("launch/*.py")),
                ("share/" + package_name + "/config", glob("config/*.yaml")),
                ("share/" + package_name + "/bt", glob("bt/*.xml"))],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Malkiabots", maintainer_email="team@example.com",
    description="Robotics Dojo 2026 mission behaviour tree", license="Apache-2.0",
    entry_points={"console_scripts": ["mission_tree = malkia_mission.mission_tree:main",
                                    "fake_inputs = malkia_mission.fake_inputs:main"]},
)
