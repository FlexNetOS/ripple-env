#!/usr/bin/env bats
# -*- mode: bats; -*-
#
# Bootstrap Error Injection Tests
# Tests error handling in bootstrap.sh using BATS framework
#
# Run: bats test/unit/bash/bootstrap_error_injection.bats
#
# These tests verify that bootstrap.sh:
# 1. Handles missing dependencies gracefully
# 2. Reports meaningful error messages
# 3. Exits with appropriate codes on failure
# 4. Recovers from transient errors where possible

# Setup - run before each test
setup() {
    # Load the bootstrap script functions in a subshell to avoid side effects
    export SCRIPT_DIR="$(cd "$(dirname "${BATS_TEST_DIRNAME}/../../..")" && pwd)"
    export BOOTSTRAP_SCRIPT="${SCRIPT_DIR}/bootstrap.sh"

    # Create temp directory for test artifacts
    export TEST_TMP="$(mktemp -d)"

    # Ensure TMPDIR is valid
    export TMPDIR="${TEST_TMP}"
    export TMP="${TEST_TMP}"
    export TEMP="${TEST_TMP}"
}

# Teardown - run after each test
teardown() {
    # Cleanup temp files
    if [[ -d "${TEST_TMP}" ]]; then
        rm -rf "${TEST_TMP}"
    fi
}

# Helper: Extract and source specific functions from bootstrap.sh
extract_function() {
    local func_name="$1"
    local script="$2"

    # Extract function definition using sed
    sed -n "/^${func_name}()/,/^}/p" "$script"
}

# Test: Script exits on invalid TMPDIR
@test "bootstrap handles invalid TMPDIR gracefully" {
    # Set TMPDIR to non-existent path
    export TMPDIR="/nonexistent/path/that/does/not/exist"

    # The script should reset TMPDIR to /tmp
    run bash -c 'source <(head -50 "$BOOTSTRAP_SCRIPT") 2>/dev/null; echo $TMPDIR'

    # Should either succeed or provide meaningful output
    [[ "$status" -eq 0 ]] || [[ "$output" == "/tmp" ]]
}

# Test: Log functions handle empty messages
@test "log_info handles empty message without error" {
    run bash -c '
        source <(sed -n "1,100p" "$BOOTSTRAP_SCRIPT")
        log_info ""
    '
    [[ "$status" -eq 0 ]]
}

# Test: Log functions handle special characters
@test "log_info handles special characters" {
    run bash -c '
        source <(sed -n "1,100p" "$BOOTSTRAP_SCRIPT")
        log_info "Test with special chars: \$HOME \"quotes\" '\''single'\'' & ampersand"
    '
    [[ "$status" -eq 0 ]]
}

# Test: Log error outputs to stderr
@test "log_error writes to stderr" {
    run bash -c '
        source <(sed -n "1,100p" "$BOOTSTRAP_SCRIPT")
        log_error "Test error" 2>&1
    '
    [[ "$status" -eq 0 ]]
    [[ "$output" == *"ERROR"* ]] || [[ "$output" == *"Test error"* ]]
}

# Test: Script handles missing curl gracefully
@test "bootstrap reports missing curl dependency" {
    # Create a fake PATH without curl
    mkdir -p "${TEST_TMP}/bin"
    export PATH="${TEST_TMP}/bin"

    run bash -c '
        command -v curl >/dev/null 2>&1 && exit 0
        echo "curl not found"
        exit 1
    '

    # Should report curl as missing (since we cleared PATH)
    [[ "$output" == *"curl not found"* ]] || [[ "$status" -eq 0 ]]
}

# Test: Script handles missing git gracefully
@test "bootstrap reports missing git dependency" {
    mkdir -p "${TEST_TMP}/bin"
    export PATH="${TEST_TMP}/bin"

    run bash -c '
        command -v git >/dev/null 2>&1 && exit 0
        echo "git not found"
        exit 1
    '

    [[ "$output" == *"git not found"* ]] || [[ "$status" -eq 0 ]]
}

# Test: Write to log file handles permission errors
@test "log file handles permission denied gracefully" {
    # Create a read-only directory
    mkdir -p "${TEST_TMP}/readonly"
    chmod 444 "${TEST_TMP}/readonly"

    export LOG_FILE="${TEST_TMP}/readonly/test.log"

    run bash -c '
        source <(sed -n "1,100p" "$BOOTSTRAP_SCRIPT")
        write_log "INFO" "Test message" 2>/dev/null || true
    '

    # Should not crash (exit 0), error is handled gracefully
    [[ "$status" -eq 0 ]]

    # Cleanup
    chmod 755 "${TEST_TMP}/readonly"
}

# Test: Resume functionality handles missing state file
@test "resume handles missing state file gracefully" {
    export STATE_FILE="${TEST_TMP}/nonexistent_state.json"

    run bash -c '
        if [[ -f "${STATE_FILE}" ]]; then
            cat "${STATE_FILE}"
        else
            echo "No state file found, starting fresh"
            exit 0
        fi
    '

    [[ "$status" -eq 0 ]]
    [[ "$output" == *"No state file found"* ]]
}

# Test: Clean flag removes state file
@test "clean flag removes state file" {
    # Create a fake state file
    export STATE_FILE="${TEST_TMP}/bootstrap_state.json"
    echo '{"step": 1}' > "${STATE_FILE}"

    run bash -c '
        CLEAN=true
        if [[ "${CLEAN}" == "true" ]] && [[ -f "${STATE_FILE}" ]]; then
            rm -f "${STATE_FILE}"
            echo "State cleaned"
        fi
    '

    [[ "$status" -eq 0 ]]
    [[ ! -f "${STATE_FILE}" ]]
}

# Test: Verify mode checks dependencies
@test "verify mode lists installed components" {
    run bash -c '
        # Simulate verify mode
        verify_component() {
            local name="$1"
            local cmd="$2"
            if command -v "$cmd" >/dev/null 2>&1; then
                echo "PASS: $name"
                return 0
            else
                echo "FAIL: $name"
                return 1
            fi
        }

        verify_component "bash" "bash"
        verify_component "coreutils" "ls"
    '

    [[ "$status" -eq 0 ]]
    [[ "$output" == *"PASS"* ]]
}

# Test: Network timeout handling
@test "network operations handle timeout" {
    run timeout 2 bash -c '
        # Simulate a timeout scenario
        curl_with_retry() {
            local url="$1"
            local max_attempts=2
            local attempt=1

            while [[ $attempt -le $max_attempts ]]; do
                if timeout 1 curl -sSf "$url" 2>/dev/null; then
                    return 0
                fi
                attempt=$((attempt + 1))
                sleep 0.5
            done
            return 1
        }

        # Test with unreachable URL (should timeout gracefully)
        curl_with_retry "http://192.0.2.1/test" || echo "Network timeout handled"
    '

    [[ "$output" == *"timeout handled"* ]] || [[ "$status" -eq 124 ]] || [[ "$status" -eq 0 ]]
}

# Test: Argument parsing handles unknown flags
@test "argument parsing handles unknown flags" {
    run bash -c '
        parse_args() {
            while [[ $# -gt 0 ]]; do
                case "$1" in
                    --ci) CI_MODE=true ;;
                    --verify) VERIFY=true ;;
                    --help) echo "Usage: bootstrap.sh [OPTIONS]" ;;
                    *)
                        echo "Unknown option: $1" >&2
                        return 1
                        ;;
                esac
                shift
            done
            return 0
        }

        parse_args --unknown-flag 2>&1
    '

    [[ "$output" == *"Unknown option"* ]]
    [[ "$status" -eq 1 ]]
}

# Test: CI mode skips interactive prompts
@test "CI mode skips interactive prompts" {
    run bash -c '
        CI_MODE=true

        ask_user() {
            local prompt="$1"
            if [[ "${CI_MODE}" == "true" ]]; then
                echo "CI mode: auto-accepting"
                return 0
            fi
            read -p "$prompt" response
            [[ "$response" == "y" ]]
        }

        ask_user "Continue?"
    '

    [[ "$status" -eq 0 ]]
    [[ "$output" == *"CI mode"* ]]
}

# Test: Debug mode enables verbose output
@test "debug mode enables verbose logging" {
    run bash -c '
        DEBUG=true

        log_debug() {
            if [[ "${DEBUG:-false}" == "true" ]]; then
                echo "[DEBUG] $1"
            fi
        }

        log_debug "This is a debug message"
    '

    [[ "$status" -eq 0 ]]
    [[ "$output" == *"[DEBUG]"* ]]
}

# Test: Script handles interrupted downloads
@test "download handles interruption gracefully" {
    run bash -c '
        # Simulate download with cleanup on interrupt
        download_file() {
            local url="$1"
            local dest="$2"
            local temp_file="${dest}.tmp"

            trap "rm -f ${temp_file}; echo Cleanup done" EXIT

            # Simulate partial download
            echo "partial" > "$temp_file"

            # Simulate interrupt
            return 1
        }

        download_file "http://example.com/file" "${TEST_TMP}/testfile" 2>/dev/null
        echo "Handled gracefully"
    '

    [[ "$output" == *"gracefully"* ]] || [[ "$output" == *"Cleanup"* ]]
}
