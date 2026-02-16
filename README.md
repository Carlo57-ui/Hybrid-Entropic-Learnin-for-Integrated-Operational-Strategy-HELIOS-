# HELIOS
**Hybrid Entropic Learning for Integrated Operational Strategy**

HELIOS es un proyecto de investigación en robótica móvil que busca combinar
aprendizaje por refuerzo profundo con algoritmos clásicos de navegación.

La idea principal es unir lo mejor de dos mundos:
- control tradicional (estable y confiable)
- aprendizaje automático (adaptable y flexible)

El resultado es un sistema híbrido capaz de navegar de forma autónoma en
entornos complejos, evitando obstáculos y tomando decisiones en tiempo real.

---

## ¿Qué hace HELIOS?

HELIOS permite que un robot móvil:

- navegue sin mapa previo
- evite obstáculos dinámicos
- optimice su trayectoria
- se adapte a cambios en el entorno
- combine decisiones aprendidas con control clásico

En lugar de reemplazar los métodos clásicos, el sistema los integra con
aprendizaje profundo para mejorar robustez y desempeño.

---

## ¿Cómo funciona?

El sistema puede fusionar:

- aprendizaje por refuerzo profundo (SAC / TD3)
- planificadores clásicos (DWA / APF / TEB)
- sensores de profundidad
- GPS + IMU

Las acciones generadas por los distintos módulos se combinan mediante un
enfoque entrópico que prioriza estabilidad y adaptabilidad.Se recomienda 
usar la combinación SAC + DWA en aplicaciones físicas.

---

## Aplicaciones

Este tipo de arquitectura puede utilizarse en:

- robots de exploración
- búsqueda y rescate
- inspección autónoma
- navegación en entornos peligrosos
- terrenos no estructurados

---

## Estado del proyecto

- simulaciones completadas
- transición a implementación física en robot real
- optimización continua del algoritmo

