import copy
import xml.etree.ElementTree as ET
from pathlib import Path

path = Path(__file__).with_name("CPAP_TestBoard_V0_2.uvprojx")
tree = ET.parse(path)
root = tree.getroot()

EXCLUDE_CM4_GROUPS = {
    "Middleware/ST25RFAL",
    "Application/User/CM7/TouchGFX/App",
    "Application/User/CM7/TouchGFX/target",
    "Application/User/CM7/TouchGFX/target/generated",
    "Application/User/CM7/Core",
    "gui",
    "generated",
    "Lib",
}

EXCLUDE_CM7_GROUPS = {
    "Application/User/CM4/Core",
    "Middlewares/MotorControl",
}


def normalize_group(group):
    """Ensure Keil group element order: GroupName, GroupOption, Files."""
    name = group.find("GroupName")
    option = group.find("GroupOption")
    files = group.find("Files")

    for child in list(group):
        group.remove(child)

    if name is not None:
        group.append(name)
    if option is not None:
        group.append(option)
    if files is not None:
        group.append(files)


def make_exclude_group_option(template_group):
    template_go = template_group.find("GroupOption") if template_group is not None else None
    if template_go is not None:
        return copy.deepcopy(template_go)

    group_option = ET.Element("GroupOption")
    common = ET.SubElement(group_option, "CommonProperty")
    ET.SubElement(common, "UseCPPCompiler").text = "0"
    ET.SubElement(common, "RVCTCodeConst").text = "0"
    ET.SubElement(common, "RVCTZI").text = "0"
    ET.SubElement(common, "RVCTOtherData").text = "0"
    ET.SubElement(common, "ModuleSelection").text = "0"
    ET.SubElement(common, "IncludeInBuild").text = "0"
    ET.SubElement(common, "AlwaysBuild").text = "2"
    ET.SubElement(common, "GenerateAssemblyFile").text = "2"
    ET.SubElement(common, "AssembleAssemblyFile").text = "2"
    ET.SubElement(common, "PublicsOnly").text = "2"
    ET.SubElement(common, "StopOnExitCode").text = "11"
    ET.SubElement(common, "CustomArgument")
    ET.SubElement(common, "IncludeLibraryModules")
    ET.SubElement(common, "ComprImg").text = "1"
    return group_option


def set_group_excluded(group, template_go):
    option = group.find("GroupOption")
    if option is None:
        option = copy.deepcopy(template_go)
        name = group.find("GroupName")
        files = group.find("Files")
        for child in list(group):
            group.remove(child)
        if name is not None:
            group.append(name)
        group.append(option)
        if files is not None:
            group.append(files)
    else:
        cp = option.find("CommonProperty")
        if cp is None:
            cp = ET.SubElement(option, "CommonProperty")
        inc = cp.find("IncludeInBuild")
        if inc is None:
            inc = ET.SubElement(cp, "IncludeInBuild")
        inc.text = "0"

    normalize_group(group)


for target in root.findall(".//Target"):
    tname = target.find("TargetName").text
    if tname.endswith("_CM4"):
        exclude_set = EXCLUDE_CM4_GROUPS
    elif tname.endswith("_CM7"):
        exclude_set = EXCLUDE_CM7_GROUPS
    else:
        continue

    template = None
    for group in target.findall("./Groups/Group"):
        inc = group.find("GroupOption/CommonProperty/IncludeInBuild")
        if inc is not None and inc.text == "0":
            template = group
            break

    template_go = make_exclude_group_option(template)

    groups_node = target.find("./Groups")
    for group in list(groups_node.findall("Group")):
        gname = group.find("GroupName")
        if gname is None:
            normalize_group(group)
            continue

        if gname.text in exclude_set or (
            tname.endswith("_CM4") and gname.text == "Middleware/ST25RFAL"
        ):
            set_group_excluded(group, template_go)
            print(f"{tname}: excluded group {gname.text}")
        else:
            normalize_group(group)

for file_path in root.findall(".//FilePath"):
    if file_path.text:
        file_path.text = file_path.text.replace("\\", "/")

ET.indent(tree, space="  ")
tree.write(path, encoding="utf-8", xml_declaration=True)
print("Updated", path)
