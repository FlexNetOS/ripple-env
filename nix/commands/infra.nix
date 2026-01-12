# Infrastructure command wrappers (IPFS, NATS, Prometheus)
# Extracted from flake.nix for modular organization
{ pkgs, ... }:

[
            # IPFS (kubo) management
            (pkgs.writeShellScriptBin "ipfs-ctl" ''
              # IPFS node management via kubo
              # Usage: ipfs-ctl [init|start|stop|status|id]
              IPFS_PATH="''${IPFS_PATH:-$HOME/.ipfs}"

              case "''${1:-status}" in
                init)
                  if [ -d "$IPFS_PATH" ]; then
                    echo "IPFS already initialized at $IPFS_PATH"
                  else
                    echo "Initializing IPFS node..."
                    ipfs init
                    echo ""
                    echo "IPFS initialized. Start with: ipfs-ctl start"
                  fi
                  ;;
                start)
                  if [ ! -d "$IPFS_PATH" ]; then
                    echo "IPFS not initialized. Run: ipfs-ctl init" >&2
                    exit 1
                  fi
                  echo "Starting IPFS daemon..."
                  echo "  API: http://127.0.0.1:5001"
                  echo "  Gateway: http://127.0.0.1:8080"
                  exec ipfs daemon "''${@:2}"
                  ;;
                stop)
                  if pkill -f "ipfs daemon"; then
                    echo "IPFS daemon stopped"
                  else
                    echo "IPFS daemon not running"
                  fi
                  ;;
                status)
                  if pgrep -f "ipfs daemon" > /dev/null; then
                    echo "IPFS daemon is running"
                    ipfs id 2>/dev/null | head -5
                  else
                    echo "IPFS daemon is not running"
                    [ -d "$IPFS_PATH" ] && echo "  Node initialized at: $IPFS_PATH"
                  fi
                  ;;
                id)
                  ipfs id
                  ;;
                *)
                  echo "Usage: ipfs-ctl [init|start|stop|status|id]"
                  echo "  init   - Initialize IPFS node"
                  echo "  start  - Start IPFS daemon"
                  echo "  stop   - Stop IPFS daemon"
                  echo "  status - Check daemon status"
                  echo "  id     - Show node identity"
                  ;;
              esac
            '')
            # NATS server management
            (pkgs.writeShellScriptBin "nats-ctl" ''
              # NATS server management
              # Usage: nats-ctl [start|stop|status|pub|sub]
              NATS_PORT="''${NATS_PORT:-4222}"
              NATS_HTTP_PORT="''${NATS_HTTP_PORT:-8222}"

              case "''${1:-status}" in
                start)
                  echo "Starting NATS server..."
                  echo "  Client: nats://127.0.0.1:$NATS_PORT"
                  echo "  HTTP:   http://127.0.0.1:$NATS_HTTP_PORT"
                  exec nats-server -p "$NATS_PORT" -m "$NATS_HTTP_PORT" "''${@:2}"
                  ;;
                stop)
                  if pkill -f "nats-server"; then
                    echo "NATS server stopped"
                  else
                    echo "NATS server not running"
                  fi
                  ;;
                status)
                  if pgrep -f "nats-server" > /dev/null; then
                    echo "NATS server is running"
                    curl -s "http://127.0.0.1:$NATS_HTTP_PORT/varz" 2>/dev/null | jq -r '.server_id // "Connected"' || echo "  (monitoring endpoint unavailable)"
                  else
                    echo "NATS server is not running"
                  fi
                  ;;
                pub)
                  # Publish a message: nats-ctl pub <subject> <message>
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: nats-ctl pub <subject> <message>" >&2
                    exit 1
                  fi
                  nats pub "$2" "$3"
                  ;;
                sub)
                  # Subscribe to a subject: nats-ctl sub <subject>
                  if [ -z "$2" ]; then
                    echo "Usage: nats-ctl sub <subject>" >&2
                    exit 1
                  fi
                  nats sub "$2"
                  ;;
                *)
                  echo "Usage: nats-ctl [start|stop|status|pub|sub]"
                  echo "  start        - Start NATS server"
                  echo "  stop         - Stop NATS server"
                  echo "  status       - Check server status"
                  echo "  pub <s> <m>  - Publish message to subject"
                  echo "  sub <s>      - Subscribe to subject"
                  echo ""
                  echo "Environment:"
                  echo "  NATS_PORT      - Client port (default: 4222)"
                  echo "  NATS_HTTP_PORT - Monitoring port (default: 8222)"
                  ;;
              esac
            '')
            # Prometheus management
            (pkgs.writeShellScriptBin "prom-ctl" ''
              # Prometheus server management
              # Usage: prom-ctl [start|stop|status|config]
              PROM_PORT="''${PROM_PORT:-9090}"
              PROM_CONFIG="''${PROM_CONFIG:-prometheus.yml}"
              PROM_DATA="''${PROM_DATA:-./prometheus-data}"

              case "''${1:-status}" in
                start)
                  if [ ! -f "$PROM_CONFIG" ]; then
                    echo "Creating default prometheus.yml..."
                    cat > "$PROM_CONFIG" << 'PROMCFG'
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']

  # ROS2 metrics (if ros2_prometheus_exporter is running)
  # - job_name: 'ros2'
  #   static_configs:
  #     - targets: ['localhost:9100']
PROMCFG
                    echo "Created $PROM_CONFIG"
                  fi
                  mkdir -p "$PROM_DATA"
                  echo "Starting Prometheus..."
                  echo "  Web UI: http://127.0.0.1:$PROM_PORT"
                  echo "  Config: $PROM_CONFIG"
                  exec prometheus --config.file="$PROM_CONFIG" --storage.tsdb.path="$PROM_DATA" --web.listen-address=":$PROM_PORT" "''${@:2}"
                  ;;
                stop)
                  if pkill -f "prometheus.*--config.file"; then
                    echo "Prometheus stopped"
                  else
                    echo "Prometheus not running"
                  fi
                  ;;
                status)
                  if pgrep -f "prometheus.*--config.file" > /dev/null; then
                    echo "Prometheus is running"
                    curl -s "http://127.0.0.1:$PROM_PORT/-/ready" && echo " - Ready"
                  else
                    echo "Prometheus is not running"
                  fi
                  ;;
                config)
                  if [ -f "$PROM_CONFIG" ]; then
                    cat "$PROM_CONFIG"
                  else
                    echo "Config file not found: $PROM_CONFIG"
                    echo "Run 'prom-ctl start' to create a default config"
                  fi
                  ;;
                *)
                  echo "Usage: prom-ctl [start|stop|status|config]"
                  echo "  start  - Start Prometheus server"
                  echo "  stop   - Stop Prometheus server"
                  echo "  status - Check server status"
                  echo "  config - Show configuration"
                  echo ""
                  echo "Environment:"
                  echo "  PROM_PORT   - Web UI port (default: 9090)"
                  echo "  PROM_CONFIG - Config file (default: prometheus.yml)"
                  echo "  PROM_DATA   - Data directory (default: ./prometheus-data)"
                  ;;
              esac
            '')
          ];
