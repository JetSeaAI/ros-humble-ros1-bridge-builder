BASE=$(pwd)
if [ "$(basename "$BASE")" = "cyclonedds-config" ]; then
	CONFIG_XML="$BASE/cyclonedds.xml"
else
	CONFIG_XML="$BASE/cyclonedds-config/cyclonedds.xml"
fi

# Check if CYCLONEDDS_URI is already set to the same value
CURRENT_URI="file://$CONFIG_XML"
if grep -q "export CYCLONEDDS_URI=" "${HOME}/.bashrc"; then
    # Extract the currently set CYCLONEDDS_URI
    SET_URI=$(grep "export CYCLONEDDS_URI=" "${HOME}/.bashrc" | tail -n 1 | cut -d'=' -f2)
    if [ "$SET_URI" != "$CURRENT_URI" ]; then
        echo "CYCLONEDDS_URI is already set to $SET_URI in .bashrc, which is different from $CURRENT_URI."
        read -p "Do you want to overwrite it? (yes/no): " choice
        if [ "$choice" = "yes" ]; then
            sed -i "s|export CYCLONEDDS_URI=.*|export CYCLONEDDS_URI=$CURRENT_URI|" "${HOME}/.bashrc"
            echo "CYCLONEDDS_URI has been updated to $CURRENT_URI in .bashrc"
        else
            echo "Keeping the existing CYCLONEDDS_URI: $SET_URI"
        fi
    else
        echo "CYCLONEDDS_URI is already set to the correct value: $CURRENT_URI in .bashrc"
    fi
else
    echo "export CYCLONEDDS_URI=$CURRENT_URI" >> "${HOME}/.bashrc"
    echo "CYCLONEDDS_URI has been set to $CURRENT_URI in .bashrc"
fi

# Print available network interfaces before input
echo "Available network interfaces:"
ip link show | awk -F': ' '/^[0-9]+:/ {print $2}'

echo "Network interfaces configured in cyclonedds.xml:"
grep -oP '(?<=<NetworkInterface name=")[^"]+' "$CONFIG_XML"

# Teaching output
echo "-----------------------------------------------------------"
echo "To add a network interface to cyclonedds.xml using this script:"
echo "1. Run this script with the network interface name as argument."
echo "   For example: ./setup_cyclonedds.sh enp0s3"
echo "2. The script will add the interface automatically under the <Interfaces> section."
echo "3. To remove an interface, use a hyphen(-) before the interface name. example: ./setup_cyclonedds.sh -wg0"
echo "-----------------------------------------------------------"
echo "-----------------------------------------------------------"
echo "Usage examples:"
echo "  Add peers: ./setup_cyclonedds.sh -ip 10.66.66.119 10.66.66.118"
echo "  Add interfaces: ./setup_cyclonedds.sh -i wg0 wg1"
echo "  Remove interfaces: ./setup_cyclonedds.sh -rm -i wg0 wg1"
echo "  Remove peers: ./setup_cyclonedds.sh -rm -ip 10.66.66.119 10.66.66.118"
echo "-----------------------------------------------------------"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -rm)
            rmflag=1
            shift
            ;;
        -ip)
            shift
            peers=()
            while [[ $# -gt 0 && "$1" != -* ]]; do
                peers+=("$1")
                shift
            done
            for peer in "${peers[@]}"; do
                if [[ ${rmflag:-0} -eq 1 ]]; then
                    if grep -q "<Peer address=\"$peer\"/>" "$CONFIG_XML"; then
                        sed -i "\|<Peer address=\"$peer\"/>|d" "$CONFIG_XML"
                        echo "Removed peer: $peer"
                    else
                        echo "Peer not found: $peer"
                    fi
                else
                    if ! grep -q "<Peer address=\"$peer\"/>" "$CONFIG_XML"; then
                        sed -i "/<\/Peers>/ i\                <Peer address=\"$peer\"/>" "$CONFIG_XML"
                        echo "Added peer: $peer"
                    else
                        echo "Peer already exists: $peer"
                    fi
                fi
            done
            rmflag=0
            ;;
        -i)
            shift
            ifaces=()
            while [[ $# -gt 0 && "$1" != -* ]]; do
                ifaces+=("$1")
                shift
            done
            for iface in "${ifaces[@]}"; do
                if [[ ${rmflag:-0} -eq 1 ]]; then
                    if grep -q "<NetworkInterface name=\"$iface\"/>" "$CONFIG_XML"; then
                        sed -i "\|<NetworkInterface name=\"$iface\"/>|d" "$CONFIG_XML"
                        echo "Removed network interface: $iface"
                    else
                        echo "Network interface not found: $iface"
                    fi
                else
                    if ! grep -q "<NetworkInterface name=\"$iface\"/>" "$CONFIG_XML"; then
                        sed -i "/<\/Interfaces>/ i\                <NetworkInterface name=\"$iface\"/>" "$CONFIG_XML"
                        echo "Added network interface: $iface"
                    else
                        echo "Network interface already exists: $iface"
                    fi
                fi
            done
            rmflag=0
            ;;
        *)
            echo "Unknown option: $1"
            shift
            ;;
    esac
done

source "${HOME}/.bashrc"