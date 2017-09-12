//
// Created by veikas on 12.08.17.
//

/** brief
 CMAKE_INSTALL_PREFIX
 Install directory used by install.
 If “make install” is invoked or INSTALL is built, this directory is prepended onto all install directories. This
 variable defaults to /usr/local on UNIX and c:/Program Files on Windows.
 On UNIX one can use the DESTDIR mechanism in order to relocate the whole installation. DESTDIR means DESTination
 DIRectory. It is commonly used by makefile users in order to install software at non-default location. It is usually
 invoked like this:
     make DESTDIR=/home/john install
 which will install the concerned software using the installation prefix, e.g. “/usr/local” prepended with the
 DESTDIR value which finally gives “/home/john/usr/local”.
 The installation prefix is also added to CMAKE_SYSTEM_PREFIX_PATH so that find_package, find_program, find_library,
 find_path, and find_file will search the prefix for other software.

 CMAKE_PREFIX_PATH
 Path used for searching by FIND_XXX(), with appropriate suffixes added.
 Specifies a path which will be used by the FIND_XXX() commands. It contains the “base” directories, the FIND_XXX()
 commands append appropriate subdirectories to the base directories. So FIND_PROGRAM() adds /bin to each of the
 directories in the path, FIND_LIBRARY() appends /lib to each of the directories, and FIND_PATH() and FIND_FILE()
 append /include . By default it is empty, it is intended to be set by the project. See also
 CMAKE_SYSTEM_PREFIX_PATH, CMAKE_INCLUDE_PATH, CMAKE_LIBRARY_PATH, CMAKE_PROGRAM_PATH.

 CMAKE_SYSTEM_PREFIX_PATH
 Path used for searching by FIND_XXX(), with appropriate suffixes added.
 Specifies a path which will be used by the FIND_XXX() commands. It contains the “base” directories, the FIND_XXX()
 commands append appropriate subdirectories to the base directories. So FIND_PROGRAM() adds /bin to each of the
 directories in the path, FIND_LIBRARY() appends /lib to each of the directories, and FIND_PATH() and FIND_FILE()
 append /include . By default this contains the standard directories for the current system, the CMAKE_INSTALL_PREFIX
 and the CMAKE_STAGING_PREFIX. It is NOT intended to be modified by the project, use CMAKE_PREFIX_PATH for this. See
 also CMAKE_SYSTEM_INCLUDE_PATH, CMAKE_SYSTEM_LIBRARY_PATH, CMAKE_SYSTEM_PROGRAM_PATH, and CMAKE_SYSTEM_IGNORE_PATH.

 CMAKE_SYSTEM_INCLUDE_PATH
 Path used for searching by FIND_FILE() and FIND_PATH().
 Specifies a path which will be used both by FIND_FILE() and FIND_PATH(). Both commands will check each of the
 contained directories for the existence of the file which is currently searched. By default it contains the standard
 directories for the current system. It is NOT intended to be modified by the project, use CMAKE_INCLUDE_PATH for this.

 CMAKE_INCLUDE_PATH
 Specifies path which will be used both by FIND_FILE() and FIND_PATH(). Both commands will check each of the
 contained directories for the existence of the file which is currently searched.

 CMAKE_SYSTEM_IGNORE_PATH
 Path to be ignored by FIND_XXX() commands. Specifies directories to be ignored by searches in FIND_XXX() commands.

 FIND_PACKAGE()
 List of paths where find_package looks for a package.
 The default is MODULE and FindPackage.cmake is searched for. If it is not found, then the find_package falls back to
 CONFIG mode. In the CONFIG mode, PackageConfig.cmake is searched for.
 <prefix>/(lib/<arch>|lib|share)/cmake/<name>*\/          (U)
 <prefix>/(lib/<arch>|lib|share)/<name>*\/                (U)
 <prefix>/(lib/<arch>|lib|share)/<name>*\/(cmake|CMake)/  (U)
 In all cases the <name> is treated as case-insensitive and corresponds to any of the names specified (<package> or
 names given by NAMES).



 */
#ifndef CPP_TUTORIALS_CMAKELISTS_H
#define CPP_TUTORIALS_CMAKELISTS_H

#endif //CPP_TUTORIALS_CMAKELISTS_H
