#!/usr/bin/python
#
# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile
import os


required_conan_version = ">=2.0.14"


class libhal_lpc40_conan(ConanFile):
    name = "libhal-lpc40"
    license = "Apache-2.0"
    homepage = "https://libhal.github.io/libhal-lpc40"
    description = ("A collection of drivers and libraries for the LPC40 "
                   "series microcontrollers from NXP")
    topics = ("arm", "microcontroller", "lpc", "lpc40", "lpc40xx", "lpc4072",
              "lpc4074", "lpc4078", "lpc4088")
    settings = "compiler", "build_type", "os", "arch"

    python_requires = "libhal-bootstrap/[^2.0.0]"
    python_requires_extend = "libhal-bootstrap.library"

    options = {
        "platform": ["ANY"],
    }
    default_options = {
        "platform": "ANY",
    }

    @property
    def _use_linker_script(self):
        return (self.options.platform == "lpc4078" or
                self.options.platform == "lpc4076" or
                self.options.platform == "lpc4088" or
                self.options.platform == "lpc4074" or
                self.options.platform == "lpc4072")

    def requirements(self):
        bootstrap = self.python_requires["libhal-bootstrap"]
        bootstrap.module.add_library_requirements(
            self,
            override_libhal_util_version="5.0.1",
            override_libhal_version="4.3.0")
        self.requires("libhal-armcortex/[^5.0.1]", transitive_headers=True)
        self.requires("ring-span-lite/[^0.7.0]", transitive_headers=True)
        self.requires("libhal-soft/[^5.1.0]")

    def add_linker_scripts_to_link_flags(self):
        platform = str(self.options.platform)
        self.cpp_info.exelinkflags = [
            "-L" + os.path.join(self.package_folder, "linker_scripts"),
            "-T" + os.path.join("libhal-lpc40", platform + ".ld"),
        ]

    def package_info(self):
        self.cpp_info.libs = ["libhal-lpc40"]
        self.cpp_info.set_property("cmake_target_name", "libhal::lpc40")

        if self.settings.os == "baremetal" and self._use_linker_script:
            self.add_linker_scripts_to_link_flags()
            self.buildenv_info.define("LIBHAL_PLATFORM",
                                      str(self.options.platform))
            self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY",
                                      "lpc40")

    def package_id(self):
        if self.info.options.get_safe("platform"):
            del self.info.options.platform
