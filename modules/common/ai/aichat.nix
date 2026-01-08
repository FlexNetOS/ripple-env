# aichat configuration
# Tiny, provider-agnostic AI CLI for development
#
# Supported providers:
#   - Claude (Anthropic)
#   - OpenAI (GPT-4, GPT-3.5)
#   - Google (Gemini)
#   - Ollama (local models)
#   - Azure OpenAI
#   - And many more...
#
# Usage:
#   ai "explain this code"
#   ai --model claude "help me debug"
#   aichat -r coder "write a function"
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkEnableOption mkOption mkIf types;
in
{
  options.programs.aichat = {
    enable = mkEnableOption "aichat AI assistant";

    package = mkOption {
      type = types.package;
      default = pkgs.aichat;
      description = "The aichat package to use";
    };

    settings = mkOption {
      type = types.attrs;
      default = { };
      description = "aichat configuration (written to config.yaml)";
      example = {
        model = "claude";
        save = true;
        highlight = true;
      };
    };
  };

  config = mkIf config.programs.aichat.enable {
    home.packages = [ config.programs.aichat.package ];

    # XDG-compliant configuration
    xdg.configFile."aichat/config.yaml" = mkIf (config.programs.aichat.settings != { }) {
      text = builtins.toJSON config.programs.aichat.settings;
    };

    # Shell aliases for AI assistance
    home.shellAliases = {
      "ai" = "aichat";
      "ai-code" = "aichat --role coder";
      "ai-explain" = "aichat --role explain";
      "ai-review" = "aichat --role reviewer";
    };
  };
}
