# 🎱 Proyecto Billar 3D con PyOpenGL  
Simulación 3D interactiva de una mesa de billar, desarrollada con **PyOpenGL** y **GLUT**, implementando sombras proyectadas (shadow projection), iluminación clásica y un sistema de cámara orbital.

Este proyecto está inspirado en un sistema solar básico pero adaptado para crear una escena 3D más compleja: una mesa de billar con bolas, sombras y controles de cámara.

---

## 🚀 Características principales

- Mesa de billar 3D modelada con OpenGL.
- Bolas con iluminación y rotación.
- **Sombras proyectadas** mediante matrices de proyección (sin shaders).
- Cámara que rota alrededor de la mesa.
- Iluminación Phong/Gouraud usando OpenGL fijo.
- Control de teclado simple.
- Código modular y fácil de expandir.

---

## 🛠️ Tecnologías utilizadas

- **Python 3.10+**
- **PyOpenGL**
- **PyOpenGL_accelerate**
- **GLUT (FreeGLUT recomendado)**

---

## 📦 Instalación

### 1. Instalar dependencias

```bash
pip install PyOpenGL PyOpenGL_accelerate
