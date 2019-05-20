import setuptools

import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

packages = setuptools.find_packages("modules")

required = []
with open("requirements.txt", "r") as freq:
    for line in freq.read().split():
        required.append(line)

setuptools.setup(
    name="raveskills",
    version="0.1.0",
    url="https://github.com/roboy/raveskills",
    author="Roboy",
    author_email="info@roboy.org",

    description="Roboy skills developed using the ravestate library.",
    long_description=long_description,
    long_description_content_type="text/markdown",

    package_dir={'': 'modules'},
    packages=packages,
    include_package_data=True,
    package_data={},  # 'ravestate_phrases_basic_en': ['en/*.yml'],

    install_requires=required,
    python_requires='>=3.6',

    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
