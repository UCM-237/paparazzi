#!/bin/bash
# Instalación de dependencias para Paparazzi UAV

echo ">>> Actualizando lista de paquetes..."
sudo apt update

echo ">>> Comprobando Python3..."
if command -v python3 &>/dev/null; then
    echo "   Python3 ya está instalado ($(python3 --version))"
else
    echo "   Instalando Python3..."
    sudo apt install -y python3
fi

echo ">>> Comprobando pip3..."
if command -v pip3 &>/dev/null; then
    echo "   pip3 ya está instalado ($(pip3 --version))"
else
    echo "   Instalando pip3..."
    sudo apt install -y python3-pip
fi

# Lista de librerías necesarias
packages=(
  pymoo
  numpy==1.24.4
  pandas
  matplotlib==3.5.1
  scikit-learn
  lxml
  myproj
  PySide2
  shapely
)

echo ">>> Comprobando librerías de Python..."
for pkg in "${packages[@]}"; do
    echo "---------------------------------------"
    echo ">>> Revisando $pkg..."
    if python3 -c "import $pkg" &>/dev/null; then
        echo "   $pkg ya está instalado"
    else
        echo "   Instalando $pkg..."
        pip3 install "$pkg"
    fi
done

echo "---------------------------------------"
echo ">>> Instalación completada con éxito"
