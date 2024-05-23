with import <nixpkgs> {};
mkShell {
  NIX_LD = lib.fileContents "${stdenv.cc}/nix-support/dynamic-linker";
  NIX_LD_LIBRARY_PATH = lib.makeLibraryPath [
    stdenv.cc.cc
    openssl
    libxml2
  ];
  nativeBuildInputs = with pkgs; [
      rustup
      espup
      espflash
      cargo
      ldproxy
    ];
}