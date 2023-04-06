workspace "Particle Sim"
   architecture "x64"
   configurations { "Debug", "Release", "Dist" }
   startproject "Yeah"
include "external.lua"
project "UnifiedParticleSim"
   kind "ConsoleApp"
   language "C++"
   cppdialect "C++20"
   targetdir "bin/%{cfg.buildcfg}"
   staticruntime "off"

   files { "src/**.h", "src/**.cpp" , "src/**.hpp", "assets/shaders/**.vert", "assets/shaders/**.frag", "assets/shaders/**.comp", "assets/shaders/**.spv","assets/shaders/**.glsl", "models/**.obj"}

   includedirs
   {
        "src",

        "vendor/glfw/include",
        "vendor/glm",
        "vendor/stb_image",
        "vendor/tiny_obj_loader",
        "%{IncludeDir.VulkanSDK}",
        "%{IncludeDir.glm}",
   }
   libdirs
   {
      "vendor/GLFW/lib-vc2022",
      "vendor/imgui/bin/Debug-windows-x86_64/ImGui",
   }
   links
   {
      "glfw3",
      "%{Library.Vulkan}",
   }

   targetdir ("bin/" .. outputdir .. "/%{prj.name}")
   objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
   openmp "On"
   filter 'files:**.frag'
      buildmessage 'Compiling %{file.relpath}'
      buildcommands {
         'glslc "%{file.relpath}" -o "assets/shaders/%{file.basename}.frag.spv"'
      }
      buildoutputs { "assets/shaders/%{file.basename}.frag.spv"}
   filter 'files:**.vert'
      buildmessage 'Compiling %{file.relpath}'
      buildcommands {
         'glslc "%{file.relpath}" -o "assets/shaders/%{file.basename}.vert.spv"'
      }
      buildoutputs { "assets/shaders/%{file.basename}.vert.spv"}
   filter 'files:**.comp'
      buildmessage 'Compiling %{file.relpath}'
      buildcommands {
         'glslc "%{file.relpath}" -o "assets/shaders/%{file.basename}.comp.spv"'
      }
      buildoutputs { "assets/shaders/%{file.basename}.comp.spv"}

   filter "system:windows"
      systemversion "latest"
      defines { "WL_PLATFORM_WINDOWS" }

   filter "configurations:Debug"
      defines { "WL_DEBUG" }
      runtime "Debug"
      symbols "On"

   filter "configurations:Release"
      defines { "WL_RELEASE" }
      runtime "Release"
      optimize "On"
      symbols "On"

   filter "configurations:Dist"
      kind "WindowedApp"
      defines { "WL_DIST" }
      runtime "Release"
      optimize "On"
      symbols "Off"

      