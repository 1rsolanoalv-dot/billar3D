# billar.py
# Pequeño juego de billar 3D en PyOpenGL con sombras proyectadas (shadow projection).
# Requisitos: PyOpenGL, numpy
# Ejecutar: python billar.py

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys, math, time
import numpy as np

# --------- Estado global ----------
win_w, win_h = 1000, 640
cam_angle_x = 25.0
cam_angle_y = -20.0
cam_dist = 18.0

mouse_down = False
mx0 = my0 = 0
dragging = False
drag_vec = np.array([0.0, 0.0, 0.0])

paused = False

# mesa: centrada en origen, plano Y = 0
TABLE_WIDTH = 10.0   # x tamaño total
TABLE_LENGTH = 18.0  # z tamaño total
TABLE_HEIGHT = 0.0   # plano en y=0
CUSHION = 0.5        # margen donde rebotan las bolas

# fisica
BALL_RADIUS = 0.4
BALL_MASS = 1.0
FRICTION = 0.992  # damping por frame
TIME_STEP = 1.0 / 60.0

light_pos = np.array([ -5.0, 10.0, 5.0, 1.0 ])  # posicion de la luz (para sombras usamos punto)

# lista de bolas: cada bola es dict con pos (np.array), vel (np.array), color (tuple)
balls = []

# bola blanca index 0
def reset_balls():
    global balls
    balls = []
    # bola blanca en la parte frontal
    white = { 'pos': np.array([0.0, BALL_RADIUS, -5.0]), 'vel': np.zeros(3), 'col': (1.0,1.0,1.0) }
    balls.append(white)

    # triángulo simple de bolas (a modo de banderilla)
    start_z = 4.0
    rows = 5
    colors = [
        (1.0,0.0,0.0), (0.0,0.0,1.0), (1.0,0.5,0.0),
        (0.0,1.0,0.0), (1.0,1.0,0.0), (0.6,0.0,0.6)
    ]
    idx = 0
    spacing = BALL_RADIUS * 2.05
    for r in range(rows):
        for c in range(r+1):
            x = (c - r/2.0) * spacing
            z = start_z + r * (spacing * 0.87)  # hex-ish packing
            col = colors[idx % len(colors)]
            balls.append({'pos': np.array([x, BALL_RADIUS, z]), 'vel': np.zeros(3), 'col': col})
            idx += 1

reset_balls()

# ---------- utilidades ----------
def length(v):
    return math.sqrt(np.dot(v,v))

def normalize(v):
    n = length(v)
    if n == 0: return v
    return v / n

# matriz de proyección de sombra sobre plano ax+by+cz+d = 0 y luz L
def shadow_matrix(plane, lightpos):
    a,b,c,d = plane
    lx,ly,lz,lw = lightpos
    mat = np.zeros((4,4), dtype=np.float32)
    dot = a*lx + b*ly + c*lz + d*lw
    mat[0,0] = dot - a*lx
    mat[1,0] =     - a*ly
    mat[2,0] =     - a*lz
    mat[3,0] =     - a*lw

    mat[0,1] =     - b*lx
    mat[1,1] = dot - b*ly
    mat[2,1] =     - b*lz
    mat[3,1] =     - b*lw

    mat[0,2] =     - c*lx
    mat[1,2] =     - c*ly
    mat[2,2] = dot - c*lz
    mat[3,2] =     - c*lw

    mat[0,3] =     - d*lx
    mat[1,3] =     - d*ly
    mat[2,3] =     - d*lz
    mat[3,3] = dot - d*lw
    return mat.transpose()  # OpenGL espera column-major

# plano Y = 0 -> 0*x + 1*y + 0*z + 0 = 0
plane = (0.0, 1.0, 0.0, 0.0)

# ---------- OpenGL inicial ----------
def init_gl():
    glClearColor(0.05, 0.07, 0.05, 1.0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # luz
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.06,0.06,0.06,1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0,1.0,0.95,1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.8,0.8,0.8,1.0])

    # material brillante para las bolas
    glMaterialfv(GL_FRONT, GL_SPECULAR, [0.8,0.8,0.8,1.0])
    glMaterialf(GL_FRONT, GL_SHININESS, 64.0)

# ---------- dibujado ----------
def draw_table():
    # mesa verde: un rectángulo plano con bordes (cushions)
    half_w = TABLE_WIDTH/2.0
    half_l = TABLE_LENGTH/2.0
    # superficie verde
    glPushMatrix()
    glTranslatef(0.0, 0.0, 0.0)
    glNormal3f(0.0, 1.0, 0.0)
    glColor3f(0.05, 0.45, 0.05)
    glBegin(GL_QUADS)
    glVertex3f(-half_w + CUSHION, 0.001, -half_l + CUSHION)
    glVertex3f( half_w - CUSHION, 0.001, -half_l + CUSHION)
    glVertex3f( half_w - CUSHION, 0.001,  half_l - CUSHION)
    glVertex3f(-half_w + CUSHION, 0.001,  half_l - CUSHION)
    glEnd()
    glPopMatrix()

    # cushions (bordes oscuros)
    glColor3f(0.12,0.07,0.03)
    glBegin(GL_QUADS)
    # left
    glNormal3f(0,0,1)
    glVertex3f(-half_w,0.0,-half_l); glVertex3f(-half_w + CUSHION,0.0,-half_l)
    glVertex3f(-half_w + CUSHION,0.6, half_l); glVertex3f(-half_w,0.6, half_l)
    # right
    glVertex3f(half_w - CUSHION,0.0,-half_l); glVertex3f(half_w,0.0,-half_l)
    glVertex3f(half_w,0.6, half_l); glVertex3f(half_w - CUSHION,0.6, half_l)
    # top
    glVertex3f(-half_w,0.0,half_l - CUSHION); glVertex3f(half_w,0.0,half_l - CUSHION)
    glVertex3f(half_w,0.6,half_l); glVertex3f(-half_w,0.6,half_l)
    # bottom
    glVertex3f(-half_w,-0.0,-half_l); glVertex3f(half_w,-0.0,-half_l)
    glVertex3f(half_w,0.6,-half_l + CUSHION); glVertex3f(-half_w,0.6,-half_l + CUSHION)
    glEnd()

def draw_ball(ball):
    glPushMatrix()
    x,y,z = ball['pos']
    glTranslatef(x, y, z)
    glColor3f(*ball['col'])
    # material brillante: specular arriba ya definido
    quad = gluNewQuadric()
    # Para suavizar normals
    glEnable(GL_NORMALIZE)
    glutSolidSphere(BALL_RADIUS, 40, 40)
    gluDeleteQuadric(quad)
    glPopMatrix()

def draw_shadow_for_ball(ball, shadow_mat):
    # aplicamos la matriz de sombra y dibujamos una versión negra achatada
    glPushMatrix()
    glMultMatrixf(shadow_mat)  # ahora cualquier cosa dibujada queda proyectada en el plano
    x,y,z = ball['pos']
    # elevamos ligeramente para evitar z-fighting y/o usamos glPolygonOffset
    glTranslatef(x, 0.001, z)
    glScalef(1.0, 0.01, 1.0)  # aplanar en Y
    glColor4f(0.0,0.0,0.0,0.6)
    glDisable(GL_LIGHTING)
    glutSolidSphere(BALL_RADIUS, 20, 20)
    glEnable(GL_LIGHTING)
    glPopMatrix()

# ---------- fisica ----------
def step_physics(dt):
    # integrar posiciones, fricción, colisiones entre bolas y paredes
    n = len(balls)
    # actualizar posiciones
    for b in balls:
        b['pos'] += b['vel'] * dt
        # rozamiento
        b['vel'] *= FRICTION

    # colisiones bola-pared (mesa rectangular en XZ)
    half_w = TABLE_WIDTH/2.0 - CUSHION - BALL_RADIUS
    half_l = TABLE_LENGTH/2.0 - CUSHION - BALL_RADIUS
    for b in balls:
        x,z = b['pos'][0], b['pos'][2]
        if x < -half_w:
            b['pos'][0] = -half_w
            b['vel'][0] *= -0.9
        if x > half_w:
            b['pos'][0] = half_w
            b['vel'][0] *= -0.9
        if z < -half_l:
            b['pos'][2] = -half_l
            b['vel'][2] *= -0.9
        if z > half_l:
            b['pos'][2] = half_l
            b['vel'][2] *= -0.9

    # colisiones bola-bola (simple respuesta elástica, iterativa)
    for i in range(n):
        for j in range(i+1, n):
            a = balls[i]; b = balls[j]
            dp = a['pos'] - b['pos']
            dp[1] = 0.0  # ignore vertical
            dist = length(dp)
            min_dist = BALL_RADIUS*2.0
            if dist < 1e-6:
                continue
            if dist < min_dist:
                # resolver solapamiento
                overlap = 0.5 * (min_dist - dist)
                dirn = dp / dist
                a['pos'] += dirn * overlap
                b['pos'] -= dirn * overlap
                # intercambio de velocidades en la dirección de colision
                va = a['vel']; vb = b['vel']
                # componente en la dirección normal
                na = np.dot(va, dirn)
                nb = np.dot(vb, dirn)
                # velocidad de intercambio (simple)
                pa = na * (BALL_MASS - BALL_MASS) + 2 * BALL_MASS * nb
                pb = nb * (BALL_MASS - BALL_MASS) + 2 * BALL_MASS * na
                # but since masses equal, swap components
                va_norm = na * dirn
                vb_norm = nb * dirn
                a['vel'] += (vb_norm - va_norm)
                b['vel'] += (va_norm - vb_norm)

# ---------- eventos input ----------
def mouse(button, state, x, y):
    global mouse_down, mx0, my0, dragging, drag_vec
    if button == GLUT_LEFT_BUTTON:
        if state == GLUT_DOWN:
            mouse_down = True
            mx0, my0 = x, y
            dragging = True
        else:
            # al soltar, aplicar fuerza a la bola blanca basada en drag
            mouse_down = False
            if dragging:
                # vector en pantalla -> mundo
                dx = x - mx0
                dz = y - my0
                # mapa sencillo: cuanto más arrastres, mayor potencia en dirección opuesta (como un taco)
                power = math.sqrt(dx*dx + dz*dz) * 0.03
                # dirección cardinal usando cámara para orientar tiro hacia escena
                # obtenemos vector desde cam hacia centro proyectado en XZ
                ang = math.radians(cam_angle_y)
                # tomar dirección en plano XZ basada en dx
                dir_x = -dx * 0.02
                dir_z = dz * 0.02
                # aplicamos sobre la bola blanca (index 0)
                if len(balls) > 0:
                    cue = balls[0]
                    # si cue está casi en reposo o siempre le damos la velocidad
                    dirv = np.array([dir_x, 0.0, dir_z])
                    if length(dirv) < 1e-4:
                        dirv = np.array([0.0, 0.0, 1.0])
                    dirv = normalize(dirv)
                    cue['vel'] += dirv * power
            dragging = False

def motion(x,y):
    # opcional: podríamos visualizar power/dirección en HUD; por simplicidad no hacemos nada
    pass

def keyboard(key, x, y):
    global cam_angle_x, cam_angle_y, cam_dist, paused
    k = key.decode('utf-8')
    if k == 'q' or ord(key) == 27:
        sys.exit(0)
    elif k == 'p':
        paused = not paused
    elif k == 'r':
        reset_balls()
    elif k == '+':
        cam_dist = max(3.0, cam_dist - 1.0)
    elif k == '-':
        cam_dist = min(60.0, cam_dist + 1.0)

def special(key, x, y):
    global cam_angle_x, cam_angle_y
    if key == GLUT_KEY_LEFT:
        cam_angle_y -= 4.0
    elif key == GLUT_KEY_RIGHT:
        cam_angle_y += 4.0
    elif key == GLUT_KEY_UP:
        cam_angle_x = min(89.0, cam_angle_x + 4.0)
    elif key == GLUT_KEY_DOWN:
        cam_angle_x = max(-89.0, cam_angle_x - 4.0)

# ---------- display ----------
last_time = time.time()
accum = 0.0

def display():
    global last_time, accum
    current = time.time()
    dt = current - last_time
    last_time = current
    # cap dt para estabilidad
    if dt > 0.05: dt = 0.05

    if not paused:
        accum += dt
        while accum >= TIME_STEP:
            step_physics(TIME_STEP)
            accum -= TIME_STEP

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # camara estilo orbit
    ex = cam_dist * math.cos(math.radians(cam_angle_y)) * math.cos(math.radians(cam_angle_x))
    ey = cam_dist * math.sin(math.radians(cam_angle_x))
    ez = cam_dist * math.sin(math.radians(cam_angle_y)) * math.cos(math.radians(cam_angle_x))
    gluLookAt(ex, ey, ez, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0)

    # actualizar posicion de la luz (misma usada en la matriz de sombra)
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos)

    # dibujar mesa y objetos
    draw_table()

    # construir matriz de sombra
    sm = shadow_matrix(plane, light_pos)

    # 1) dibujar sombras (sin depth write para blending apropiado)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT)
    glDisable(GL_DEPTH_TEST)  # dibujamos sombras encima pero usamos proyección y transparencia
    # dibujamos cada sombra
    for b in balls:
        draw_shadow_for_ball(b, sm)
    glEnable(GL_DEPTH_TEST)
    glPopAttrib()
    glDisable(GL_BLEND)

    # 2) dibujar bolas con iluminación
    for b in balls:
        draw_ball(b)

    # HUD simple: potencia si arrastrando
    if dragging or mouse_down:
        # si quisiéramos mostrar, podríamos dibujar texto, pero GLUT bitmap strings son feos; omitido
        pass

    glutSwapBuffers()
    glutPostRedisplay()

def reshape(w,h):
    global win_w, win_h
    win_w, win_h = w,h
    glViewport(0,0,w,h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(w)/float(h if h>0 else 1), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH)
    glutInitWindowSize(win_w, win_h)
    glutCreateWindow(b"Billar 3D - sombras por proyeccion (OpenGL fijo)")
    init_gl()
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutMouseFunc(mouse)
    glutMotionFunc(motion)
    glutKeyboardFunc(keyboard)
    glutSpecialFunc(special)
    print("Controles: arrastra con el mouse y suelta para golpear. flechas: camara. p: pausa. r: reiniciar.")
    glutMainLoop()

if __name__ == '__main__':
    main()
