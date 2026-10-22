# Mallas del modelo

Las seis piezas `base_link.stl`, `camera_1.stl`, `castor_1.stl`,
`lidar_1.stl`, `l_wheel_1.stl` y `r_wheel_1.stl` proceden de Fusion.
Sus vértices están en milímetros, en el sistema de coordenadas del CAD.
El URDF aplica escala 0,001 y las transformaciones de cada articulación.

`wheel_contact.stl` es una superficie de revolución unitaria, con 96 sectores
alrededor de Y y 16 bandas entre sus polos. Se obtiene de
`(cos(latitud) cos(angulo), sin(latitud), cos(latitud) sin(angulo))`.
El escalado `(0.0375, 0.01, 0.0375)` genera el contacto redondeado de las
ruedas: radio exterior de 37,5 mm y anchura de 20 mm. Se usa únicamente
como colisión; las mallas visibles y las inercias de las ruedas son las originales.
