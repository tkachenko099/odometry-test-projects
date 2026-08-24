from setuptools import find_packages, setup

package_name = "ugv_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UGV Nav Stand",
    maintainer_email="dev@example.com",
    description="Plotly/Dash analysis dashboard (online + offline).",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "dashboard = ugv_dashboard.app:main",
            "sim-dashboard = ugv_dashboard.interactive:main",
        ],
    },
)
