from pathlib import Path

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
HEADER_SCRIPT="""#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

"""
HEADER_C="""/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

"""
HEADER_H="""/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
"""
HEADER_LUA="""--------------------------------------------------------------------------------------
--
-- Copyright (c) Contributors to the Open 3D Engine Project.
-- For complete copyright and license terms please see the LICENSE at the root of this distribution.
--
-- SPDX-License-Identifier: Apache-2.0 OR MIT
--
--
--
----------------------------------------------------------------------------------------------------

"""
root=  "/home/michal/github/o3de-ros2-gem/"

required_headers = {
    ".cmake":HEADER_SCRIPT,
    "CMakeLists.txt":HEADER_SCRIPT,
    ".cpp":HEADER_C,
    ".h":HEADER_H,
    ".sh":HEADER_SCRIPT,
    ".py":HEADER_SCRIPT,
    ".lua": HEADER_LUA,
    ".azsli": HEADER_C,
    
}

files = []


def checkHeader(required_header, file_to_test):
    with open(file_to_test, 'r') as file:
        data = file.read()
    if data.startswith(required_header):
        return True
    return False

if __name__ == "__main__":
    for elem in Path(root).rglob('*.*'):
        if not any(part.startswith('.')for part in elem.parts):
            files.append(elem)
    files_without_lic = 0;   

    for file in files:
        extension = file.suffix
        filename = file.name
    
        if (filename in required_headers.keys()):
            header = required_headers[filename]
            isOk = checkHeader(header, file)
            if (isOk):
                print ("%s %sOK%s" %(file,bcolors.OKGREEN,bcolors.ENDC))
            else:
                print ("%s %sHEADER MISSING%s" %(file,bcolors.WARNING,bcolors.ENDC))
                files_without_lic+=1
       
         
    for file in files:
        extension = file.suffix
        if (extension in required_headers.keys()):
            header = required_headers[extension]
            isOk = checkHeader(header, file)
            if (isOk):
                print ("%s %sOK%s" %(file,bcolors.OKGREEN,bcolors.ENDC))
            else:
                print ("%s %sHEADER MISSING%s" %(file,bcolors.WARNING,bcolors.ENDC))   
                files_without_lic+=1 
  
    

    for file in files:
        extension = file.suffix
        filename = file.name
        if (extension not in required_headers.keys() and filename not in required_headers.keys()):
            print ("%s %sNO CHECK%s" %(file,bcolors.OKCYAN,bcolors.ENDC))  
            
    if (files_without_lic > 0):
        print ("%sThere are files %d with missing license %s" %(bcolors.FAIL,files_without_lic,bcolors.ENDC))


# check root directory
    desirable_files = ['gem.json', 'CMakeLists.txt', 'gem.json', "README.md"]
    for elem in Path(root).glob('*.*'):
        if elem.name not in desirable_files:
            if not any(part.startswith('.')for part in elem.parts):
                print ("%s Undesirable file %s %s" %(bcolors.FAIL,elem,bcolors.ENDC))

    f_out = open("/tmp/codebase.cpp", 'w')
    files.sort();
    for file in files:
        extension = file.suffix
        filename = file.name
        if (extension in ['.cpp', '.h']):
            f_out.write("////////////////////////////////////////////////////////////////////////////////////\n")
            f_out.write("////  %s \n" % file)
            f_out.write("////////////////////////////////////////////////////////////////////////////////////\n")

            with open(file, 'r') as filer:
                f_out.write(filer.read())
            f_out.write("\n")