#!/usr/bin/env python3
"""Generates InitiatorDrone.xcodeproj from the source tree.

The project file is generated rather than hand-maintained so that adding a
Swift file never means editing a pbxproj by hand and never produces a merge
conflict in one. Re-run after adding or removing files:

    python3 Scripts/generate_xcodeproj.py

Produces an Xcode 14-compatible project (objectVersion 56) with two targets:
the iOS app and its unit-test bundle.
"""

import hashlib
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))

PROJECT_NAME = "InitiatorDrone"
APP_TARGET = "InitiatorDrone"
TEST_TARGET = "InitiatorDroneTests"
DEPLOYMENT_TARGET = "16.0"
SWIFT_VERSION = "5.0"

PROJECT_DIR = os.path.join(ROOT, f"{PROJECT_NAME}.xcodeproj")

# Signing lives outside the generated project.
#
# Setting a team in Xcode's Signing & Capabilities pane writes into
# project.pbxproj, which this script overwrites — so the team would vanish the
# next time anyone added a source file. Instead it is read from
# Scripts/signing.local (git-ignored) or the environment, and baked in on every
# regeneration.
SIGNING_FILE = os.path.join(HERE, "signing.local")
DEFAULT_BUNDLE_ID = "com.initiatordrone.app"


def load_signing():
    """Reads DEVELOPMENT_TEAM and PRODUCT_BUNDLE_IDENTIFIER, if configured."""
    values = {}
    if os.path.isfile(SIGNING_FILE):
        with open(SIGNING_FILE, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                values[key.strip()] = value.strip()

    # The environment wins, so a one-off build can override the file.
    for key in ("DEVELOPMENT_TEAM", "PRODUCT_BUNDLE_IDENTIFIER"):
        if os.environ.get(key):
            values[key] = os.environ[key]

    values.setdefault("PRODUCT_BUNDLE_IDENTIFIER", DEFAULT_BUNDLE_ID)
    return values

_counter = [0]


def uid(tag=""):
    """Deterministic 24-hex-character object identifiers.

    Derived from a stable digest rather than Python's `hash`, which is salted
    per process: regenerating must produce a byte-identical project so the file
    does not churn in git every time someone adds a source file.
    """
    _counter[0] += 1
    digest = hashlib.md5(f"{tag}#{_counter[0]}".encode("utf-8")).hexdigest()
    return digest[:24].upper()


def swift_files(directory):
    found = []
    for base, dirs, names in os.walk(directory):
        dirs[:] = sorted(d for d in dirs if not d.startswith("."))
        for name in sorted(names):
            if name.endswith(".swift"):
                found.append(os.path.join(base, name))
    return found


class Node:
    """A group in the Xcode navigator."""

    def __init__(self, name, path=None):
        self.name = name
        self.path = path
        self.id = uid("group:" + name)
        self.children = {}
        self.files = []
        # Extra child group ids to splice in, e.g. the Resources group.
        self.extra_children = []


def build_tree(root_name, root_path, files, group_path=None):
    """Builds a group tree.

    `root_path` is where the files live on disk; `group_path` is what goes into
    the project file, and must stay relative so the project is not pinned to
    one machine's checkout location.
    """
    root = Node(root_name, group_path if group_path is not None else root_name)
    for absolute in files:
        relative = os.path.relpath(absolute, root_path)
        parts = relative.split(os.sep)
        node = root
        for part in parts[:-1]:
            if part not in node.children:
                node.children[part] = Node(part, part)
            node = node.children[part]
        node.files.append((parts[-1], absolute))
    return root


def emit_groups(node, file_refs, lines):
    for child in node.children.values():
        emit_groups(child, file_refs, lines)

    children = []
    for child in node.children.values():
        children.append(f"\t\t\t\t{child.id} /* {child.name} */,")
    for child_id, child_name in node.extra_children:
        children.append(f"\t\t\t\t{child_id} /* {child_name} */,")
    for name, absolute in node.files:
        children.append(f"\t\t\t\t{file_refs[absolute]} /* {name} */,")

    lines.append(f"\t\t{node.id} /* {node.name} */ = {{")
    lines.append("\t\t\tisa = PBXGroup;")
    lines.append("\t\t\tchildren = (")
    lines.extend(children)
    lines.append("\t\t\t);")
    if node.path:
        lines.append(f'\t\t\tpath = "{node.path}";')
    else:
        lines.append(f'\t\t\tname = "{node.name}";')
    lines.append("\t\t\tsourceTree = \"<group>\";")
    lines.append("\t\t};")


def generate():
    signing = load_signing()
    team = signing.get("DEVELOPMENT_TEAM")
    bundle_id = signing["PRODUCT_BUNDLE_IDENTIFIER"]

    app_sources = swift_files(os.path.join(ROOT, "InitiatorDrone"))
    test_sources = swift_files(os.path.join(ROOT, "Tests"))

    if not app_sources:
        sys.exit("No app sources found")

    info_plist = os.path.join(ROOT, "InitiatorDrone", "Resources", "Info.plist")
    fixtures_dir = os.path.join(ROOT, "InitiatorDrone", "Resources", "Fixtures")

    # ---------------------------------------------------------------- ids
    project_id = uid("project")
    app_target_id = uid("target:app")
    test_target_id = uid("target:test")
    main_group_id = uid("mainGroup")
    products_group_id = uid("products")
    app_product_id = uid("product:app")
    test_product_id = uid("product:test")
    fixtures_ref_id = uid("fixtures")
    plist_ref_id = uid("plist")

    app_sources_phase = uid("phase:appSources")
    app_resources_phase = uid("phase:appResources")
    app_frameworks_phase = uid("phase:appFrameworks")
    test_sources_phase = uid("phase:testSources")
    test_frameworks_phase = uid("phase:testFrameworks")

    dependency_id = uid("dependency")
    container_proxy_id = uid("containerProxy")

    project_config_list = uid("configList:project")
    app_config_list = uid("configList:app")
    test_config_list = uid("configList:test")

    configs = {
        ("project", "Debug"): uid("config:project:Debug"),
        ("project", "Release"): uid("config:project:Release"),
        ("app", "Debug"): uid("config:app:Debug"),
        ("app", "Release"): uid("config:app:Release"),
        ("test", "Debug"): uid("config:test:Debug"),
        ("test", "Release"): uid("config:test:Release"),
    }

    file_refs = {path: uid("ref:" + path) for path in app_sources + test_sources}
    build_files = {path: uid("build:" + path) for path in app_sources + test_sources}
    fixtures_build_id = uid("build:fixtures")

    app_tree = build_tree(
        "InitiatorDrone", os.path.join(ROOT, "InitiatorDrone"), app_sources, group_path="InitiatorDrone"
    )
    test_tree = build_tree("Tests", os.path.join(ROOT, "Tests"), test_sources, group_path="Tests")

    # The Resources group holds Info.plist and the fixtures folder. It has to
    # hang off the app group, or its file references are unreachable from the
    # main group and Xcode will not show or copy them.
    resources_group_id = uid("group:Resources")
    app_tree.extra_children.append((resources_group_id, "Resources"))

    lines = []
    add = lines.append

    add("// !$*UTF8*$!")
    add("{")
    add("\tarchiveVersion = 1;")
    add("\tclasses = {")
    add("\t};")
    add("\tobjectVersion = 56;")
    add("\tobjects = {")

    # ------------------------------------------------------- PBXBuildFile
    add("")
    add("/* Begin PBXBuildFile section */")
    for path in app_sources + test_sources:
        name = os.path.basename(path)
        add(
            f"\t\t{build_files[path]} /* {name} in Sources */ = {{isa = PBXBuildFile; "
            f"fileRef = {file_refs[path]} /* {name} */; }};"
        )
    add(
        f"\t\t{fixtures_build_id} /* Fixtures in Resources */ = {{isa = PBXBuildFile; "
        f"fileRef = {fixtures_ref_id} /* Fixtures */; }};"
    )
    add("/* End PBXBuildFile section */")

    # ------------------------------------------ PBXContainerItemProxy
    add("")
    add("/* Begin PBXContainerItemProxy section */")
    add(f"\t\t{container_proxy_id} /* PBXContainerItemProxy */ = {{")
    add("\t\t\tisa = PBXContainerItemProxy;")
    add(f"\t\t\tcontainerPortal = {project_id} /* Project object */;")
    add("\t\t\tproxyType = 1;")
    add(f"\t\t\tremoteGlobalIDString = {app_target_id};")
    add(f"\t\t\tremoteInfo = {APP_TARGET};")
    add("\t\t};")
    add("/* End PBXContainerItemProxy section */")

    # ---------------------------------------------------- PBXFileReference
    add("")
    add("/* Begin PBXFileReference section */")
    for path in app_sources + test_sources:
        name = os.path.basename(path)
        add(
            f"\t\t{file_refs[path]} /* {name} */ = {{isa = PBXFileReference; "
            f"lastKnownFileType = sourcecode.swift; path = \"{name}\"; sourceTree = \"<group>\"; }};"
        )
    add(
        f"\t\t{plist_ref_id} /* Info.plist */ = {{isa = PBXFileReference; "
        "lastKnownFileType = text.plist.xml; path = Info.plist; sourceTree = \"<group>\"; };"
    )
    # A folder reference, so the Fixtures directory is copied into the bundle
    # with its structure intact rather than flattened.
    add(
        f"\t\t{fixtures_ref_id} /* Fixtures */ = {{isa = PBXFileReference; "
        "lastKnownFileType = folder; path = Fixtures; sourceTree = \"<group>\"; };"
    )
    add(
        f"\t\t{app_product_id} /* {APP_TARGET}.app */ = {{isa = PBXFileReference; "
        f"explicitFileType = wrapper.application; includeInIndex = 0; path = \"{APP_TARGET}.app\"; "
        "sourceTree = BUILT_PRODUCTS_DIR; };"
    )
    add(
        f"\t\t{test_product_id} /* {TEST_TARGET}.xctest */ = {{isa = PBXFileReference; "
        f"explicitFileType = wrapper.cfbundle; includeInIndex = 0; path = \"{TEST_TARGET}.xctest\"; "
        "sourceTree = BUILT_PRODUCTS_DIR; };"
    )
    add("/* End PBXFileReference section */")

    # ---------------------------------------------------- PBXFrameworksBuildPhase
    add("")
    add("/* Begin PBXFrameworksBuildPhase section */")
    for phase in (app_frameworks_phase, test_frameworks_phase):
        add(f"\t\t{phase} /* Frameworks */ = {{")
        add("\t\t\tisa = PBXFrameworksBuildPhase;")
        add("\t\t\tbuildActionMask = 2147483647;")
        add("\t\t\tfiles = (")
        add("\t\t\t);")
        add("\t\t\trunOnlyForDeploymentPostprocessing = 0;")
        add("\t\t};")
    add("/* End PBXFrameworksBuildPhase section */")

    # -------------------------------------------------------------- PBXGroup
    add("")
    add("/* Begin PBXGroup section */")

    group_lines = []
    emit_groups(app_tree, file_refs, group_lines)
    emit_groups(test_tree, file_refs, group_lines)
    lines.extend(group_lines)

    add(f"\t\t{resources_group_id} /* Resources */ = {{")
    add("\t\t\tisa = PBXGroup;")
    add("\t\t\tchildren = (")
    add(f"\t\t\t\t{plist_ref_id} /* Info.plist */,")
    add(f"\t\t\t\t{fixtures_ref_id} /* Fixtures */,")
    add("\t\t\t);")
    add("\t\t\tpath = Resources;")
    add("\t\t\tsourceTree = \"<group>\";")
    add("\t\t};")

    add(f"\t\t{products_group_id} /* Products */ = {{")
    add("\t\t\tisa = PBXGroup;")
    add("\t\t\tchildren = (")
    add(f"\t\t\t\t{app_product_id} /* {APP_TARGET}.app */,")
    add(f"\t\t\t\t{test_product_id} /* {TEST_TARGET}.xctest */,")
    add("\t\t\t);")
    add("\t\t\tname = Products;")
    add("\t\t\tsourceTree = \"<group>\";")
    add("\t\t};")

    add(f"\t\t{main_group_id} = {{")
    add("\t\t\tisa = PBXGroup;")
    add("\t\t\tchildren = (")
    add(f"\t\t\t\t{app_tree.id} /* InitiatorDrone */,")
    add(f"\t\t\t\t{test_tree.id} /* Tests */,")
    add(f"\t\t\t\t{products_group_id} /* Products */,")
    add("\t\t\t);")
    add("\t\t\tsourceTree = \"<group>\";")
    add("\t\t};")
    add("/* End PBXGroup section */")

    # ----------------------------------------------------------- PBXNativeTarget
    add("")
    add("/* Begin PBXNativeTarget section */")
    add(f"\t\t{app_target_id} /* {APP_TARGET} */ = {{")
    add("\t\t\tisa = PBXNativeTarget;")
    add(f"\t\t\tbuildConfigurationList = {app_config_list};")
    add("\t\t\tbuildPhases = (")
    add(f"\t\t\t\t{app_sources_phase} /* Sources */,")
    add(f"\t\t\t\t{app_frameworks_phase} /* Frameworks */,")
    add(f"\t\t\t\t{app_resources_phase} /* Resources */,")
    add("\t\t\t);")
    add("\t\t\tbuildRules = (")
    add("\t\t\t);")
    add("\t\t\tdependencies = (")
    add("\t\t\t);")
    add(f"\t\t\tname = {APP_TARGET};")
    add(f"\t\t\tproductName = {APP_TARGET};")
    add(f"\t\t\tproductReference = {app_product_id} /* {APP_TARGET}.app */;")
    add("\t\t\tproductType = \"com.apple.product-type.application\";")
    add("\t\t};")

    add(f"\t\t{test_target_id} /* {TEST_TARGET} */ = {{")
    add("\t\t\tisa = PBXNativeTarget;")
    add(f"\t\t\tbuildConfigurationList = {test_config_list};")
    add("\t\t\tbuildPhases = (")
    add(f"\t\t\t\t{test_sources_phase} /* Sources */,")
    add(f"\t\t\t\t{test_frameworks_phase} /* Frameworks */,")
    add("\t\t\t);")
    add("\t\t\tbuildRules = (")
    add("\t\t\t);")
    add("\t\t\tdependencies = (")
    add(f"\t\t\t\t{dependency_id} /* PBXTargetDependency */,")
    add("\t\t\t);")
    add(f"\t\t\tname = {TEST_TARGET};")
    add(f"\t\t\tproductName = {TEST_TARGET};")
    add(f"\t\t\tproductReference = {test_product_id} /* {TEST_TARGET}.xctest */;")
    add("\t\t\tproductType = \"com.apple.product-type.bundle.unit-test\";")
    add("\t\t};")
    add("/* End PBXNativeTarget section */")

    # --------------------------------------------------------------- PBXProject
    add("")
    add("/* Begin PBXProject section */")
    add(f"\t\t{project_id} /* Project object */ = {{")
    add("\t\t\tisa = PBXProject;")
    add("\t\t\tattributes = {")
    add("\t\t\t\tBuildIndependentTargetsInParallel = 1;")
    add("\t\t\t\tLastSwiftUpdateCheck = 1500;")
    add("\t\t\t\tLastUpgradeCheck = 1500;")
    add("\t\t\t\tTargetAttributes = {")
    add(f"\t\t\t\t\t{app_target_id} = {{")
    add("\t\t\t\t\t\tCreatedOnToolsVersion = 15.0;")
    add("\t\t\t\t\t};")
    add(f"\t\t\t\t\t{test_target_id} = {{")
    add("\t\t\t\t\t\tCreatedOnToolsVersion = 15.0;")
    add(f"\t\t\t\t\t\tTestTargetID = {app_target_id};")
    add("\t\t\t\t\t};")
    add("\t\t\t\t};")
    add("\t\t\t};")
    add(f"\t\t\tbuildConfigurationList = {project_config_list};")
    add("\t\t\tcompatibilityVersion = \"Xcode 14.0\";")
    add("\t\t\tdevelopmentRegion = en;")
    add("\t\t\thasScannedForEncodings = 0;")
    add("\t\t\tknownRegions = (")
    add("\t\t\t\ten,")
    add("\t\t\t\tBase,")
    add("\t\t\t);")
    add(f"\t\t\tmainGroup = {main_group_id};")
    add(f"\t\t\tproductRefGroup = {products_group_id} /* Products */;")
    add("\t\t\tprojectDirPath = \"\";")
    add("\t\t\tprojectRoot = \"\";")
    add("\t\t\ttargets = (")
    add(f"\t\t\t\t{app_target_id} /* {APP_TARGET} */,")
    add(f"\t\t\t\t{test_target_id} /* {TEST_TARGET} */,")
    add("\t\t\t);")
    add("\t\t};")
    add("/* End PBXProject section */")

    # ------------------------------------------------- PBXResourcesBuildPhase
    add("")
    add("/* Begin PBXResourcesBuildPhase section */")
    add(f"\t\t{app_resources_phase} /* Resources */ = {{")
    add("\t\t\tisa = PBXResourcesBuildPhase;")
    add("\t\t\tbuildActionMask = 2147483647;")
    add("\t\t\tfiles = (")
    add(f"\t\t\t\t{fixtures_build_id} /* Fixtures in Resources */,")
    add("\t\t\t);")
    add("\t\t\trunOnlyForDeploymentPostprocessing = 0;")
    add("\t\t};")
    add("/* End PBXResourcesBuildPhase section */")

    # --------------------------------------------------- PBXSourcesBuildPhase
    add("")
    add("/* Begin PBXSourcesBuildPhase section */")
    for phase, sources in ((app_sources_phase, app_sources), (test_sources_phase, test_sources)):
        add(f"\t\t{phase} /* Sources */ = {{")
        add("\t\t\tisa = PBXSourcesBuildPhase;")
        add("\t\t\tbuildActionMask = 2147483647;")
        add("\t\t\tfiles = (")
        for path in sources:
            name = os.path.basename(path)
            add(f"\t\t\t\t{build_files[path]} /* {name} in Sources */,")
        add("\t\t\t);")
        add("\t\t\trunOnlyForDeploymentPostprocessing = 0;")
        add("\t\t};")
    add("/* End PBXSourcesBuildPhase section */")

    # ------------------------------------------------------ PBXTargetDependency
    add("")
    add("/* Begin PBXTargetDependency section */")
    add(f"\t\t{dependency_id} /* PBXTargetDependency */ = {{")
    add("\t\t\tisa = PBXTargetDependency;")
    add(f"\t\t\ttarget = {app_target_id} /* {APP_TARGET} */;")
    add(f"\t\t\ttargetProxy = {container_proxy_id} /* PBXContainerItemProxy */;")
    add("\t\t};")
    add("/* End PBXTargetDependency section */")

    # ------------------------------------------------- XCBuildConfiguration
    add("")
    add("/* Begin XCBuildConfiguration section */")

    def project_settings(debug):
        settings = {
            "ALWAYS_SEARCH_USER_PATHS": "NO",
            "CLANG_ENABLE_MODULES": "YES",
            "CLANG_ENABLE_OBJC_ARC": "YES",
            "COPY_PHASE_STRIP": "NO",
            "ENABLE_STRICT_OBJC_MSGSEND": "YES",
            "GCC_C_LANGUAGE_STANDARD": "gnu11",
            "IPHONEOS_DEPLOYMENT_TARGET": DEPLOYMENT_TARGET,
            "SDKROOT": "iphoneos",
            "SWIFT_VERSION": SWIFT_VERSION,
            "ENABLE_USER_SCRIPT_SANDBOXING": "YES",
            "SWIFT_EMIT_LOC_STRINGS": "YES",
        }
        if debug:
            settings.update({
                "DEBUG_INFORMATION_FORMAT": "dwarf",
                "ENABLE_TESTABILITY": "YES",
                "GCC_OPTIMIZATION_LEVEL": "0",
                "ONLY_ACTIVE_ARCH": "YES",
                "SWIFT_ACTIVE_COMPILATION_CONDITIONS": "DEBUG",
                "SWIFT_OPTIMIZATION_LEVEL": "-Onone",
                "GCC_PREPROCESSOR_DEFINITIONS": '"DEBUG=1 $(inherited)"',
            })
        else:
            settings.update({
                "DEBUG_INFORMATION_FORMAT": '"dwarf-with-dsym"',
                "ENABLE_NS_ASSERTIONS": "NO",
                "SWIFT_COMPILATION_MODE": "wholemodule",
                "VALIDATE_PRODUCT": "YES",
            })
        return settings

    def app_settings():
        settings = {
            "ASSETCATALOG_COMPILER_GLOBAL_ACCENT_COLOR_NAME": "AccentColor",
            "CODE_SIGN_STYLE": "Automatic",
            "CURRENT_PROJECT_VERSION": "1",
            "GENERATE_INFOPLIST_FILE": "NO",
            "INFOPLIST_FILE": "InitiatorDrone/Resources/Info.plist",
            "LD_RUNPATH_SEARCH_PATHS": '(\n\t\t\t\t\t"$(inherited)",\n\t\t\t\t\t"@executable_path/Frameworks",\n\t\t\t\t)',
            "MARKETING_VERSION": "1.0",
            "PRODUCT_BUNDLE_IDENTIFIER": bundle_id,
            "PRODUCT_NAME": '"$(TARGET_NAME)"',
            "SWIFT_EMIT_LOC_STRINGS": "YES",
            "TARGETED_DEVICE_FAMILY": '"1,2"',
        }
        if team:
            settings["DEVELOPMENT_TEAM"] = team
        return settings

    def test_settings():
        settings = {
            "BUNDLE_LOADER": '"$(TEST_HOST)"',
            "CODE_SIGN_STYLE": "Automatic",
            "CURRENT_PROJECT_VERSION": "1",
            "GENERATE_INFOPLIST_FILE": "YES",
            "MARKETING_VERSION": "1.0",
            "PRODUCT_BUNDLE_IDENTIFIER": bundle_id + ".tests",
            "PRODUCT_NAME": '"$(TARGET_NAME)"',
            "TARGETED_DEVICE_FAMILY": '"1,2"',
            "TEST_HOST": f'"$(BUILT_PRODUCTS_DIR)/{APP_TARGET}.app/$(BUNDLE_EXECUTABLE_FOLDER_PATH)/{APP_TARGET}"',
        }
        if team:
            settings["DEVELOPMENT_TEAM"] = team
        return settings

    def emit_config(config_id, name, settings):
        add(f"\t\t{config_id} /* {name} */ = {{")
        add("\t\t\tisa = XCBuildConfiguration;")
        add("\t\t\tbuildSettings = {")
        for key in sorted(settings):
            add(f"\t\t\t\t{key} = {settings[key]};")
        add("\t\t\t};")
        add(f"\t\t\tname = {name};")
        add("\t\t};")

    emit_config(configs[("project", "Debug")], "Debug", project_settings(True))
    emit_config(configs[("project", "Release")], "Release", project_settings(False))
    emit_config(configs[("app", "Debug")], "Debug", app_settings())
    emit_config(configs[("app", "Release")], "Release", app_settings())
    emit_config(configs[("test", "Debug")], "Debug", test_settings())
    emit_config(configs[("test", "Release")], "Release", test_settings())
    add("/* End XCBuildConfiguration section */")

    # -------------------------------------------- XCConfigurationList
    add("")
    add("/* Begin XCConfigurationList section */")
    for list_id, scope, label in (
        (project_config_list, "project", f"PBXProject \"{PROJECT_NAME}\""),
        (app_config_list, "app", f"PBXNativeTarget \"{APP_TARGET}\""),
        (test_config_list, "test", f"PBXNativeTarget \"{TEST_TARGET}\""),
    ):
        add(f"\t\t{list_id} /* Build configuration list for {label} */ = {{")
        add("\t\t\tisa = XCConfigurationList;")
        add("\t\t\tbuildConfigurations = (")
        add(f"\t\t\t\t{configs[(scope, 'Debug')]} /* Debug */,")
        add(f"\t\t\t\t{configs[(scope, 'Release')]} /* Release */,")
        add("\t\t\t);")
        add("\t\t\tdefaultConfigurationIsVisible = 0;")
        add("\t\t\tdefaultConfigurationName = Release;")
        add("\t\t};")
    add("/* End XCConfigurationList section */")

    add("\t};")
    add(f"\trootObject = {project_id} /* Project object */;")
    add("}")

    # ---------------------------------------------------------------- write
    if os.path.exists(PROJECT_DIR):
        shutil.rmtree(PROJECT_DIR)
    os.makedirs(os.path.join(PROJECT_DIR, "project.xcworkspace", "xcshareddata"))
    os.makedirs(os.path.join(PROJECT_DIR, "xcshareddata", "xcschemes"))

    pbxproj_path = os.path.join(PROJECT_DIR, "project.pbxproj")
    with open(pbxproj_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")

    with open(
        os.path.join(PROJECT_DIR, "project.xcworkspace", "contents.xcworkspacedata"),
        "w",
        encoding="utf-8",
    ) as handle:
        handle.write(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Workspace version = "1.0">\n'
            '   <FileRef location = "self:">\n'
            "   </FileRef>\n"
            "</Workspace>\n"
        )

    with open(
        os.path.join(
            PROJECT_DIR, "project.xcworkspace", "xcshareddata", "IDEWorkspaceChecks.plist"
        ),
        "w",
        encoding="utf-8",
    ) as handle:
        handle.write(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" '
            '"http://www.apple.com/DTDs/PropertyList-1.0.dtd">\n'
            '<plist version="1.0">\n<dict>\n'
            "\t<key>IDEDidComputeMac32BitWarning</key>\n\t<true/>\n"
            "</dict>\n</plist>\n"
        )

    scheme = SCHEME_TEMPLATE.format(
        app_target_id=app_target_id,
        test_target_id=test_target_id,
        app_target=APP_TARGET,
        test_target=TEST_TARGET,
        project_name=PROJECT_NAME,
    )
    with open(
        os.path.join(PROJECT_DIR, "xcshareddata", "xcschemes", f"{APP_TARGET}.xcscheme"),
        "w",
        encoding="utf-8",
    ) as handle:
        handle.write(scheme)

    print(f"Generated {os.path.relpath(PROJECT_DIR, ROOT)}")
    print(f"  app target:  {len(app_sources)} Swift files")
    print(f"  test target: {len(test_sources)} Swift files")
    print(f"  bundle id:   {bundle_id}")
    if team:
        print(f"  team:        {team}")
    else:
        print("  team:        not set — see Scripts/signing.local.example")
    if not os.path.isdir(fixtures_dir):
        print("  WARNING: Resources/Fixtures is missing; run generate_fixtures.py")
    if not os.path.isfile(info_plist):
        print("  WARNING: Resources/Info.plist is missing")

    # `plutil` parses the OpenStep plist format pbxproj uses, so this catches a
    # malformed file here rather than when Xcode refuses to open it.
    result = subprocess.run(
        ["plutil", "-lint", pbxproj_path], capture_output=True, text=True
    )
    if result.returncode != 0:
        print(result.stdout or result.stderr)
        sys.exit("project.pbxproj failed validation")
    print("  project.pbxproj: valid")


SCHEME_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<Scheme
   LastUpgradeVersion = "1500"
   version = "1.7">
   <BuildAction
      parallelizeBuildables = "YES"
      buildImplicitDependencies = "YES">
      <BuildActionEntries>
         <BuildActionEntry
            buildForTesting = "YES"
            buildForRunning = "YES"
            buildForProfiling = "YES"
            buildForArchiving = "YES"
            buildForAnalyzing = "YES">
            <BuildableReference
               BuildableIdentifier = "primary"
               BlueprintIdentifier = "{app_target_id}"
               BuildableName = "{app_target}.app"
               BlueprintName = "{app_target}"
               ReferencedContainer = "container:{project_name}.xcodeproj">
            </BuildableReference>
         </BuildActionEntry>
      </BuildActionEntries>
   </BuildAction>
   <TestAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
      selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
      shouldUseLaunchSchemeArgsEnv = "YES">
      <Testables>
         <TestableReference
            skipped = "NO">
            <BuildableReference
               BuildableIdentifier = "primary"
               BlueprintIdentifier = "{test_target_id}"
               BuildableName = "{test_target}.xctest"
               BlueprintName = "{test_target}"
               ReferencedContainer = "container:{project_name}.xcodeproj">
            </BuildableReference>
         </TestableReference>
      </Testables>
   </TestAction>
   <LaunchAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
      selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
      launchStyle = "0"
      useCustomWorkingDirectory = "NO"
      ignoresPersistentStateOnLaunch = "NO"
      debugDocumentVersioning = "YES"
      debugServiceExtension = "internal"
      allowLocationSimulation = "YES">
      <BuildableProductRunnable
         runnableDebuggingMode = "0">
         <BuildableReference
            BuildableIdentifier = "primary"
            BlueprintIdentifier = "{app_target_id}"
            BuildableName = "{app_target}.app"
            BlueprintName = "{app_target}"
            ReferencedContainer = "container:{project_name}.xcodeproj">
         </BuildableReference>
      </BuildableProductRunnable>
   </LaunchAction>
   <ProfileAction
      buildConfiguration = "Release"
      shouldUseLaunchSchemeArgsEnv = "YES"
      savedToolIdentifier = ""
      useCustomWorkingDirectory = "NO"
      debugDocumentVersioning = "YES">
      <BuildableProductRunnable
         runnableDebuggingMode = "0">
         <BuildableReference
            BuildableIdentifier = "primary"
            BlueprintIdentifier = "{app_target_id}"
            BuildableName = "{app_target}.app"
            BlueprintName = "{app_target}"
            ReferencedContainer = "container:{project_name}.xcodeproj">
         </BuildableReference>
      </BuildableProductRunnable>
   </ProfileAction>
   <AnalyzeAction
      buildConfiguration = "Debug">
   </AnalyzeAction>
   <ArchiveAction
      buildConfiguration = "Release"
      revealArchiveInOrganizer = "YES">
   </ArchiveAction>
</Scheme>
"""


if __name__ == "__main__":
    generate()
