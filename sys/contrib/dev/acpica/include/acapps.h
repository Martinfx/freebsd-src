/******************************************************************************
 *
 * Module Name: acapps - common include for ACPI applications/tools
 *
 *****************************************************************************/

/******************************************************************************
 *
 * 1. Copyright Notice
 *
 * Some or all of this work - Copyright (c) 1999 - 2025, Intel Corp.
 * All rights reserved.
 *
 * 2. License
 *
 * 2.1. This is your license from Intel Corp. under its intellectual property
 * rights. You may have additional license terms from the party that provided
 * you this software, covering your right to use that party's intellectual
 * property rights.
 *
 * 2.2. Intel grants, free of charge, to any person ("Licensee") obtaining a
 * copy of the source code appearing in this file ("Covered Code") an
 * irrevocable, perpetual, worldwide license under Intel's copyrights in the
 * base code distributed originally by Intel ("Original Intel Code") to copy,
 * make derivatives, distribute, use and display any portion of the Covered
 * Code in any form, with the right to sublicense such rights; and
 *
 * 2.3. Intel grants Licensee a non-exclusive and non-transferable patent
 * license (with the right to sublicense), under only those claims of Intel
 * patents that are infringed by the Original Intel Code, to make, use, sell,
 * offer to sell, and import the Covered Code and derivative works thereof
 * solely to the minimum extent necessary to exercise the above copyright
 * license, and in no event shall the patent license extend to any additions
 * to or modifications of the Original Intel Code. No other license or right
 * is granted directly or by implication, estoppel or otherwise;
 *
 * The above copyright and patent license is granted only if the following
 * conditions are met:
 *
 * 3. Conditions
 *
 * 3.1. Redistribution of Source with Rights to Further Distribute Source.
 * Redistribution of source code of any substantial portion of the Covered
 * Code or modification with rights to further distribute source must include
 * the above Copyright Notice, the above License, this list of Conditions,
 * and the following Disclaimer and Export Compliance provision. In addition,
 * Licensee must cause all Covered Code to which Licensee contributes to
 * contain a file documenting the changes Licensee made to create that Covered
 * Code and the date of any change. Licensee must include in that file the
 * documentation of any changes made by any predecessor Licensee. Licensee
 * must include a prominent statement that the modification is derived,
 * directly or indirectly, from Original Intel Code.
 *
 * 3.2. Redistribution of Source with no Rights to Further Distribute Source.
 * Redistribution of source code of any substantial portion of the Covered
 * Code or modification without rights to further distribute source must
 * include the following Disclaimer and Export Compliance provision in the
 * documentation and/or other materials provided with distribution. In
 * addition, Licensee may not authorize further sublicense of source of any
 * portion of the Covered Code, and must include terms to the effect that the
 * license from Licensee to its licensee is limited to the intellectual
 * property embodied in the software Licensee provides to its licensee, and
 * not to intellectual property embodied in modifications its licensee may
 * make.
 *
 * 3.3. Redistribution of Executable. Redistribution in executable form of any
 * substantial portion of the Covered Code or modification must reproduce the
 * above Copyright Notice, and the following Disclaimer and Export Compliance
 * provision in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3.4. Intel retains all right, title, and interest in and to the Original
 * Intel Code.
 *
 * 3.5. Neither the name Intel nor any other trademark owned or controlled by
 * Intel shall be used in advertising or otherwise to promote the sale, use or
 * other dealings in products derived from or relating to the Covered Code
 * without prior written authorization from Intel.
 *
 * 4. Disclaimer and Export Compliance
 *
 * 4.1. INTEL MAKES NO WARRANTY OF ANY KIND REGARDING ANY SOFTWARE PROVIDED
 * HERE. ANY SOFTWARE ORIGINATING FROM INTEL OR DERIVED FROM INTEL SOFTWARE
 * IS PROVIDED "AS IS," AND INTEL WILL NOT PROVIDE ANY SUPPORT, ASSISTANCE,
 * INSTALLATION, TRAINING OR OTHER SERVICES. INTEL WILL NOT PROVIDE ANY
 * UPDATES, ENHANCEMENTS OR EXTENSIONS. INTEL SPECIFICALLY DISCLAIMS ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * 4.2. IN NO EVENT SHALL INTEL HAVE ANY LIABILITY TO LICENSEE, ITS LICENSEES
 * OR ANY OTHER THIRD PARTY, FOR ANY LOST PROFITS, LOST DATA, LOSS OF USE OR
 * COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES, OR FOR ANY INDIRECT,
 * SPECIAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THIS AGREEMENT, UNDER ANY
 * CAUSE OF ACTION OR THEORY OF LIABILITY, AND IRRESPECTIVE OF WHETHER INTEL
 * HAS ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES. THESE LIMITATIONS
 * SHALL APPLY NOTWITHSTANDING THE FAILURE OF THE ESSENTIAL PURPOSE OF ANY
 * LIMITED REMEDY.
 *
 * 4.3. Licensee shall not export, either directly or indirectly, any of this
 * software or system incorporating such software without first obtaining any
 * required license or other approval from the U. S. Department of Commerce or
 * any other agency or department of the United States Government. In the
 * event Licensee exports any such software from the United States or
 * re-exports any such software from a foreign destination, Licensee shall
 * ensure that the distribution and export/re-export of the software is in
 * compliance with all laws, regulations, orders, or other restrictions of the
 * U.S. Export Administration Regulations. Licensee agrees that neither it nor
 * any of its subsidiaries will export/re-export any technical data, process,
 * software, or service, directly or indirectly, to any country for which the
 * United States government or any agency thereof requires an export license,
 * other governmental approval, or letter of assurance, without first obtaining
 * such license, approval or letter.
 *
 *****************************************************************************
 *
 * Alternatively, you may choose to be licensed under the terms of the
 * following license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, you may choose to be licensed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 *****************************************************************************/

#ifndef _ACAPPS
#define _ACAPPS

#ifdef ACPI_USE_STANDARD_HEADERS
#include <sys/stat.h>
#endif /* ACPI_USE_STANDARD_HEADERS */

/* Common info for tool signons */

#define ACPICA_NAME                 "Intel ACPI Component Architecture"
#define ACPICA_COPYRIGHT            "Copyright (c) 2000 - 2025 Intel Corporation"

#if ACPI_MACHINE_WIDTH == 64
#define ACPI_WIDTH          " (64-bit version)"

#elif ACPI_MACHINE_WIDTH == 32
#define ACPI_WIDTH          " (32-bit version)"

#else
#error unknown ACPI_MACHINE_WIDTH
#define ACPI_WIDTH          " (unknown bit width, not 32 or 64)"

#endif

/* Macros for signons and file headers */

#define ACPI_COMMON_SIGNON(UtilityName) \
    "\n%s\n%s version %8.8X\n%s\n\n", \
    ACPICA_NAME, \
    UtilityName, ((UINT32) ACPI_CA_VERSION), \
    ACPICA_COPYRIGHT

#define ACPI_COMMON_HEADER(UtilityName, Prefix) \
    "%s%s\n%s%s version %8.8X%s\n%s%s\n%s\n", \
    Prefix, ACPICA_NAME, \
    Prefix, UtilityName, ((UINT32) ACPI_CA_VERSION), ACPI_WIDTH, \
    Prefix, ACPICA_COPYRIGHT, \
    Prefix

#define ACPI_COMMON_BUILD_TIME \
    "Build date/time: %s %s\n", __DATE__, __TIME__

/* Macros for usage messages */

#define ACPI_USAGE_HEADER(Usage) \
    printf ("Usage: %s\nOptions:\n", Usage);

#define ACPI_USAGE_TEXT(Description) \
    printf (Description);

#define ACPI_OPTION(Name, Description) \
    printf ("  %-20s%s\n", Name, Description);


/* Check for unexpected exceptions */

#define ACPI_CHECK_STATUS(Name, Status, Expected) \
    if (Status != Expected) \
    { \
        AcpiOsPrintf ("Unexpected %s from %s (%s-%d)\n", \
            AcpiFormatException (Status), #Name, _AcpiModuleName, __LINE__); \
    }

/* Check for unexpected non-AE_OK errors */


#define ACPI_CHECK_OK(Name, Status)   ACPI_CHECK_STATUS (Name, Status, AE_OK);

#define FILE_SUFFIX_DISASSEMBLY     "dsl"
#define FILE_SUFFIX_BINARY_TABLE    ".dat" /* Needs the dot */


/* acfileio */

ACPI_STATUS
AcGetAllTablesFromFile (
    char                    *Filename,
    UINT8                   GetOnlyAmlTables,
    ACPI_NEW_TABLE_DESC     **ReturnListHead);

void
AcDeleteTableList (
    ACPI_NEW_TABLE_DESC     *ListHead);

BOOLEAN
AcIsFileBinary (
    FILE                    *File);

ACPI_STATUS
AcValidateTableHeader (
    FILE                    *File,
    long                    TableOffset);


/* Values for GetOnlyAmlTables */

#define ACPI_GET_ONLY_AML_TABLES    TRUE
#define ACPI_GET_ALL_TABLES         FALSE


/*
 * getopt
 */
int
AcpiGetopt(
    int                     argc,
    char                    **argv,
    char                    *opts);

int
AcpiGetoptArgument (
    int                     argc,
    char                    **argv);

extern int                  AcpiGbl_Optind;
extern int                  AcpiGbl_Opterr;
extern int                  AcpiGbl_SubOptChar;
extern char                 *AcpiGbl_Optarg;


/*
 * cmfsize - Common get file size function
 */
UINT32
CmGetFileSize (
    ACPI_FILE               File);


/*
 * adwalk
 */
void
AcpiDmCrossReferenceNamespace (
    ACPI_PARSE_OBJECT       *ParseTreeRoot,
    ACPI_NAMESPACE_NODE     *NamespaceRoot,
    ACPI_OWNER_ID           OwnerId);

void
AcpiDmDumpTree (
    ACPI_PARSE_OBJECT       *Origin);

void
AcpiDmFindOrphanMethods (
    ACPI_PARSE_OBJECT       *Origin);

void
AcpiDmFinishNamespaceLoad (
    ACPI_PARSE_OBJECT       *ParseTreeRoot,
    ACPI_NAMESPACE_NODE     *NamespaceRoot,
    ACPI_OWNER_ID           OwnerId);

void
AcpiDmConvertParseObjects (
    ACPI_PARSE_OBJECT       *ParseTreeRoot,
    ACPI_NAMESPACE_NODE     *NamespaceRoot);


/*
 * adfile
 */
ACPI_STATUS
AdInitialize (
    void);

char *
FlGenerateFilename (
    char                    *InputFilename,
    char                    *Suffix);

ACPI_STATUS
FlSplitInputPathname (
    char                    *InputPath,
    char                    **OutDirectoryPath,
    char                    **OutFilename);

char *
FlGetFileBasename (
    char                    *FilePathname);

char *
AdGenerateFilename (
    char                    *Prefix,
    char                    *TableId);

void
AdWriteTable (
    ACPI_TABLE_HEADER       *Table,
    UINT32                  Length,
    char                    *TableName,
    char                    *OemTableId);

#endif /* _ACAPPS */
