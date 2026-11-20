"""Dev index for the Dev build workflow (.github/workflows/dev-build.yml).

  dev_index.py version <stable index> <platform.txt> <dev index> <dev tags file>
  dev_index.py add <dev index> <version> <zip> <url> <release url> <keep> <boards.txt> <stable index>
  dev_index.py remove <dev index> <version>
"""
import hashlib
import json
import os
import re
import sys


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save(path, index):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(index, f, indent=2)
        f.write("\n")


def release_key(version):
    m = re.match(r"(\d+)\.(\d+)\.(\d+)", version)
    return tuple(int(x) for x in m.groups()) if m else (0, 0, 0)


def dev_number(version):
    m = re.search(r"-dev\.(\d+)$", version)
    return int(m.group(1)) if m else 0


def newest_stable(package):
    stable = [p for p in package["platforms"] if "-" not in p["version"]]
    return max(stable, key=lambda p: release_key(p["version"]))


def cmd_version(stable_path, platform_txt, dev_path, tags_path):
    versions = [p["version"] for p in load(stable_path)["packages"][0]["platforms"]]
    with open(platform_txt, encoding="utf-8") as f:
        m = re.search(r"^version=(\S+)", f.read(), re.M)
    if m:
        versions.append(m.group(1))
    major, minor, patch = max(release_key(v) for v in versions if "-" not in v)
    base = f"{major}.{minor}.{patch + 1}"

    used = []
    if os.path.exists(dev_path):
        used += [p["version"] for p in load(dev_path)["packages"][0]["platforms"]]
    if os.path.exists(tags_path):
        with open(tags_path, encoding="utf-8") as f:
            used += [t.strip()[len("dev-"):] for t in f if t.startswith("dev-")]
    last = max((dev_number(v) for v in used if v.startswith(base + "-dev.")), default=0)
    print(f"{base}-dev.{last + 1}")


def cmd_add(dev_path, version, zip_path, url, release_url, keep, boards_txt, stable_path):
    stable = load(stable_path)["packages"][0]
    latest = newest_stable(stable)
    deps = latest["toolsDependencies"]
    needed = {(d["name"], d["version"]) for d in deps}

    if os.path.exists(dev_path):
        index = load(dev_path)
    else:
        index = {"packages": [{k: v for k, v in stable.items() if k not in ("platforms", "tools")}]}
        index["packages"][0].update(platforms=[], tools=[])
    package = index["packages"][0]
    have = {(t["name"], t["version"]) for t in package["tools"]}
    package["tools"] += [t for t in stable["tools"]
                         if (t["name"], t["version"]) in needed and (t["name"], t["version"]) not in have]

    with open(zip_path, "rb") as f:
        data = f.read()
    with open(boards_txt, encoding="utf-8") as f:
        boards = [{"name": m.group(1).strip()} for m in re.finditer(r"^\w+\.name=(.+)$", f.read(), re.M)]
    entry = {
        "name": latest["name"],
        "architecture": latest["architecture"],
        "version": version,
        "category": latest.get("category", "Contributed"),
        "help": {"online": release_url},
        "url": url,
        "archiveFileName": os.path.basename(zip_path),
        "checksum": "SHA-256:" + hashlib.sha256(data).hexdigest().upper(),
        "size": str(len(data)),
        "boards": boards,
        "toolsDependencies": deps,
    }

    builds = [p for p in package["platforms"] if p["version"] != version] + [entry]
    builds.sort(key=lambda p: (release_key(p["version"]), dev_number(p["version"])))
    keep = int(keep)
    dropped = builds[:-keep] if len(builds) > keep else []
    package["platforms"] = builds[len(dropped):]
    save(dev_path, index)
    for p in dropped:
        print(p["version"])


def cmd_remove(dev_path, version):
    index = load(dev_path)
    package = index["packages"][0]
    builds = [p for p in package["platforms"] if p["version"] != version]
    if len(builds) == len(package["platforms"]):
        sys.exit(f"{version} is not in the dev index")
    package["platforms"] = builds
    save(dev_path, index)


if __name__ == "__main__":
    commands = {"version": (cmd_version, 4), "add": (cmd_add, 8), "remove": (cmd_remove, 2)}
    if len(sys.argv) < 2 or sys.argv[1] not in commands or len(sys.argv) - 2 != commands[sys.argv[1]][1]:
        sys.exit(__doc__)
    command, _ = commands[sys.argv[1]]
    command(*sys.argv[2:])
