from setuptools import find_packages, setup

package_name = "results_server"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="paz",
    maintainer_email="niccolo.pasetto@studenti.unipd.it",
    description="Debug node to provide a simple web server with the annotated frames of the C.V. nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "results_server = " + package_name + ".results_server:main"
        ],
    },
)
