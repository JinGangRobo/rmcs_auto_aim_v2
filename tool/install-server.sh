#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
RES_DIR="${SCRIPT_DIR}/res"
INSTALL_ROOT="/opt/autoaim"
BIN_DIR="${INSTALL_ROOT}/bin"
TARGET_RES_DIR="${INSTALL_ROOT}/res"
DEFAULT_REMOTE_HOST="remote"
MEDIAMTX_VERSION="v1.18.2"

usage() {
    cat <<EOF
Usage:
  $(basename "$0") local
  $(basename "$0") remote [host] [--target-arch <arm64|amd64>]
  $(basename "$0") <payload_dir>

For 'remote', the target architecture is resolved in this order:
  1. --target-arch <arm64|amd64>
  2. "# RMCS_TARGET_ARCH=<arch>" in ~/.ssh/config (set by 'set-remote --arch')
  3. ssh <host> uname -m
EOF
}

require_command() {
    local command_name="$1"

    if ! command -v "${command_name}" >/dev/null 2>&1; then
        printf 'Missing command: %s\n' "${command_name}" >&2
        exit 1
    fi
}

run_privileged() {
    if command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        "$@"
    fi
}

# Convert a machine/arch string to the normalized form used by .script/sync-remote.
normalize_arch() {
    case "$1" in
    x86_64 | amd64) printf 'amd64\n' ;;
    aarch64 | arm64) printf 'arm64\n' ;;
    armv7l | armv7) printf 'armv7\n' ;;
    *) return 1 ;;
    esac
}

# Convert a normalized arch to the mediamtx release platform suffix.
mediamtx_arch() {
    case "$1" in
    amd64) printf 'linux_amd64\n' ;;
    arm64) printf 'linux_arm64\n' ;;
    armv7) printf 'linux_armv7\n' ;;
    *) return 1 ;;
    esac
}

local_arch() {
    normalize_arch "$(uname -m)"
}

# Read "RMCS_TARGET_ARCH=<arch>" from the given host block in ~/.ssh/config,
# falling back to the "Host remote" block when the host has no own block.
read_target_arch_from_ssh_config() {
    local host="$1"
    local ssh_config="${HOME}/.ssh/config"

    [[ -f "${ssh_config}" ]] || return 1

    local candidate value normalized
    for candidate in "${host}" remote; do
        [[ -n "${candidate}" ]] || continue

        value="$(
            awk -v host="${candidate}" '
                $1 == "Host" {
                    inblock = 0
                    for (i = 2; i <= NF; i++)
                        if ($i == host)
                            inblock = 1
                    next
                }
                inblock && match($0, /RMCS_TARGET_ARCH=[^ \t]+/) {
                    print substr($0, RSTART + 17, RLENGTH - 17)
                    exit
                }
            ' "${ssh_config}"
        )"
        [[ -n "${value}" ]] || continue

        if normalized="$(normalize_arch "${value}")"; then
            printf '%s\n' "${normalized}"
            return 0
        fi

        printf 'Warning: ignoring invalid RMCS_TARGET_ARCH=%s for host %s.\n' \
            "${value}" "${candidate}" >&2
    done

    return 1
}

# Detect the architecture of the remote host over SSH.
detect_remote_arch() {
    local host="$1"
    local machine

    machine="$(
        ssh -o BatchMode=yes -o ConnectTimeout=10 "${host}" 'uname -m' 2>/dev/null \
            | tr -d '\r\n'
    )" || return 1
    [[ -n "${machine}" ]] || return 1

    normalize_arch "${machine}"
}

# Resolve the target architecture: explicit flag > ssh config > ssh detection.
resolve_target_arch() {
    local host="$1"
    local explicit="$2"
    local value

    if [[ -n "${explicit}" ]]; then
        printf '%s\n' "${explicit}"
        return 0
    fi

    if value="$(read_target_arch_from_ssh_config "${host}")"; then
        printf '%s\n' "${value}"
        return 0
    fi

    if value="$(detect_remote_arch "${host}")"; then
        printf '%s\n' "${value}"
        return 0
    fi

    printf 'Error: cannot determine the architecture of remote host "%s".\n' "${host}" >&2
    printf '  Pass --target-arch <arm64|amd64>, or configure it with:\n' >&2
    printf '    set-remote %s --arch <arm64|amd64>\n' "${host}" >&2
    exit 1
}

# Download the mediamtx release for the given (normalized) target architecture.
download_mediamtx() {
    local target_arch="$1"
    local output_dir="$2"

    local platform
    if ! platform="$(mediamtx_arch "${target_arch}")"; then
        printf 'Unsupported architecture: %s\n' "${target_arch}" >&2
        exit 1
    fi

    local archive="mediamtx_${MEDIAMTX_VERSION}_${platform}.tar.gz"

    curl -L --fail \
        "https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/${archive}" \
        -o "${output_dir}/${archive}"
    tar -xzf "${output_dir}/${archive}" -C "${output_dir}"
}

append_path_to_zshrc() {
    local line='export PATH="/opt/autoaim/bin:$PATH"'
    local zshrc_path="${HOME}/.zshrc"

    touch "${zshrc_path}"
    if ! grep -Fq "${line}" "${zshrc_path}"; then
        printf '\n%s\n' "${line}" >>"${zshrc_path}"
    fi
}

# The locally installed mediamtx may only be reused when it matches the target
# architecture; otherwise the target-arch release must be downloaded.
installed_runtime_matches() {
    local target_arch="$1"
    local current_arch="$2"

    [[ "${target_arch}" == "${current_arch}" ]] || return 1

    [[ -f "${BIN_DIR}/mediamtx" ]] &&
        "${BIN_DIR}/mediamtx" --version 2>/dev/null | grep -Fq "${MEDIAMTX_VERSION}"
}

install_runtime_dependencies() {
    local packages=(
        curl
        python3
        gstreamer1.0-tools
        gstreamer1.0-plugins-base
        gstreamer1.0-plugins-good
        gstreamer1.0-plugins-ugly
    )
    local missing_packages=()
    local package

    for package in "${packages[@]}"; do
        if ! dpkg -s "${package}" >/dev/null 2>&1; then
            missing_packages+=("${package}")
        fi
    done

    if [[ ${#missing_packages[@]} -eq 0 ]]; then
        return
    fi

    run_privileged apt update
    run_privileged apt install -y "${missing_packages[@]}"
}

install_from() {
    local payload_dir="$1"

    [[ -d "${payload_dir}" ]] || {
        printf 'Missing payload directory: %s\n' "${payload_dir}" >&2
        exit 1
    }

    [[ -f "${payload_dir}/bin/mediamtx" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/bin/mediamtx" >&2
        exit 1
    }

    [[ -f "${payload_dir}/bin/start-streamer" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/bin/start-streamer" >&2
        exit 1
    }

    [[ -f "${payload_dir}/res/mediamtx.yml" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/res/mediamtx.yml" >&2
        exit 1
    }

    [[ -f "${payload_dir}/res/playing.html" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/res/playing.html" >&2
        exit 1
    }

    install_runtime_dependencies

    run_privileged mkdir -p "${BIN_DIR}" "${TARGET_RES_DIR}"
    run_privileged install -m 755 "${payload_dir}/bin/mediamtx" "${BIN_DIR}/mediamtx"
    run_privileged install -m 755 "${payload_dir}/bin/start-streamer" "${BIN_DIR}/start-streamer"
    run_privileged install -m 644 "${payload_dir}/res/mediamtx.yml" "${TARGET_RES_DIR}/mediamtx.yml"
    run_privileged install -m 644 "${payload_dir}/res/playing.html" "${TARGET_RES_DIR}/playing.html"

    append_path_to_zshrc

    cat <<EOF
Install completed.

Binary directory:
  ${BIN_DIR}

Resource directory:
  ${TARGET_RES_DIR}

Next step:
  start-streamer
EOF
}

prepare_payload() {
    local payload_dir="$1"
    local target_arch="$2"

    mkdir -p "${payload_dir}/bin" "${payload_dir}/res"

    install -m 755 "${RES_DIR}/start-streamer" "${payload_dir}/bin/start-streamer"
    install -m 644 "${RES_DIR}/mediamtx.yml" "${payload_dir}/res/mediamtx.yml"
    install -m 644 "${RES_DIR}/playing.html" "${payload_dir}/res/playing.html"

    local current_arch
    current_arch="$(local_arch)" || current_arch=""

    if installed_runtime_matches "${target_arch}" "${current_arch}"; then
        printf 'Reusing installed mediamtx runtime: %s\n' "${MEDIAMTX_VERSION}"
        install -m 755 "${BIN_DIR}/mediamtx" "${payload_dir}/bin/mediamtx"
        return
    fi

    printf 'Downloading mediamtx runtime: %s (%s)\n' "${MEDIAMTX_VERSION}" "${target_arch}"
    download_mediamtx "${target_arch}" "${payload_dir}/bin"
}

install_local() {
    local payload_dir
    local target_arch

    if ! target_arch="$(local_arch)"; then
        printf 'Unsupported local architecture: %s\n' "$(uname -m)" >&2
        exit 1
    fi

    payload_dir="$(mktemp -d /tmp/autoaim-install-local.XXXXXX)"
    prepare_payload "${payload_dir}" "${target_arch}"
    install_from "${payload_dir}"
}

install_remote() {
    local remote_host="$1"
    local target_arch="$2"
    local local_payload_dir
    local remote_payload_dir="/tmp/autoaim-install-payload"

    local_payload_dir="$(mktemp -d /tmp/autoaim-install-remote.XXXXXX)"
    prepare_payload "${local_payload_dir}" "${target_arch}"

    ssh "${remote_host}" "rm -rf '${remote_payload_dir}' && mkdir -p '${remote_payload_dir}'"
    scp -r "${local_payload_dir}/." "${remote_host}:${remote_payload_dir}/"
    scp "$0" "${remote_host}:${remote_payload_dir}/install-server.sh"
    ssh "${remote_host}" "bash '${remote_payload_dir}/install-server.sh' '${remote_payload_dir}'"
}

main() {
    local mode=""
    local remote_host=""
    local target_arch=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
        --target-arch)
            if [[ $# -lt 2 ]]; then
                printf 'Missing value for --target-arch\n' >&2
                usage >&2
                exit 1
            fi
            target_arch="$2"
            shift 2
            ;;
        --target-arch=*)
            target_arch="${1#*=}"
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        --*)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 1
            ;;
        *)
            if [[ -z "${mode}" ]]; then
                mode="$1"
            elif [[ -z "${remote_host}" ]]; then
                remote_host="$1"
            else
                printf 'Too many arguments: %s\n' "$1" >&2
                usage >&2
                exit 1
            fi
            shift
            ;;
        esac
    done

    if [[ -n "${target_arch}" && "${target_arch}" != "arm64" && "${target_arch}" != "amd64" ]]; then
        printf 'Unsupported --target-arch: %s (use arm64 or amd64)\n' "${target_arch}" >&2
        exit 1
    fi

    case "${mode}" in
    local)
        if [[ -n "${remote_host}" ]]; then
            printf 'Error: "local" does not take a host argument.\n' >&2
            exit 1
        fi
        if [[ -n "${target_arch}" ]]; then
            printf 'Note: ignoring --target-arch for local install (using the local architecture).\n' >&2
        fi
        install_local
        ;;
    remote)
        require_command ssh
        require_command scp

        if [[ -z "${remote_host}" ]]; then
            remote_host="${DEFAULT_REMOTE_HOST}"
        fi

        target_arch="$(resolve_target_arch "${remote_host}" "${target_arch}")"
        printf 'Target architecture for %s: %s\n' "${remote_host}" "${target_arch}"
        install_remote "${remote_host}" "${target_arch}"
        ;;
    "")
        usage >&2
        exit 1
        ;;
    *)
        if [[ -d "${mode}" && -z "${remote_host}" && -z "${target_arch}" ]]; then
            install_from "${mode}"
            return
        fi
        usage >&2
        exit 1
        ;;
    esac
}

main "$@"
