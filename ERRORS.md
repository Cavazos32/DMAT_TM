# ERRORS.md — DMAT-TM

## 1. Autoridad y alcance

Catálogo normativo propuesto para aprobación del propietario. Una vez aprobado, ningún agente puede modificarlo, agregar códigos, cambiar mensajes, instrucciones, estilos o rutas sin autorización expresa. Aplica a todos los errores y warnings, en todos los estados y temas. Esta especificación no autoriza cambios de firmware, secuencias de máquina, tiempos, condiciones de seguridad ni movimientos.

## 2. Contrato de presentación

- Banner y recuadro de diagnóstico: `E### Módulo: Descripción.` exactamente según la tabla. No agregar prefijos Error/Warning, pasos, nombres de funciones, sufijos técnicos, instrucciones ni texto libre. El único separador de dos puntos es el que sigue al módulo.
- El código, módulo, clasificación, descripción e instrucción son campos separados. Los lados L/R del PreAlimentador forman parte del módulo visible. Main es el nombre visible del orquestador; no renombrar archivos ni funciones internas.
- Mostrar las instrucciones exclusivamente en su sección correspondiente, no en el diagnóstico. La instrucción específica del código es de diagnóstico inicial; no sustituye los requisitos de cada etapa de recuperación. Mantenerla identificable y separada de la guía dinámica mientras sea pertinente. No concatenar mensajes improvisados.
- Errores y comunicación comparten el mismo estilo rojo. Usar una sola variable de color semántico por clasificación y tema, compartida por banner, indicador y recuadro. No cambiar colores de controles ajenos a este alcance. Conservar la familia tipográfica vigente; centralizar tamaño, peso, radio, bordes, alineación, espaciado y estilos de botones por componente. No crear variantes por código. Mantener contraste legible en tema claro y oscuro.
- El botón de acceso se denomina `Abrir` al aparecer el diagnóstico; conservar las acciones específicas de etapas posteriores. Comunicación abre Parámetros → Conexión; no crear un popup de recuperación para ese acceso.
- Si coinciden varios códigos, conservar cada código distinto, aunque coincidan las descripciones; no alterar la prioridad ni el enclavamiento actuales. No duplicar el mismo código.
- No mostrar identificadores internos ni valores de `detail` sin traducir. Conservarlos en diagnóstico técnico. Si no se conoce la causa, usar el fallback autorizado sin inventar una causa. E046 solo cuando no se pueda identificar el motivo concreto del Reset bloqueado; usar el código específico cuando ya esté confirmado. E000 queda reservado como fallback; no eliminar su soporte.

## 3. Catálogo completo (52 códigos)

Las siguientes descripciones e instrucciones son literales. Las instrucciones de comunicación pertenecen al apartado Conexión, no a una ventana de recuperación convencional. Error = recuperación actual convencional; Comunicación = acceso especial. No inferir acciones físicas a partir de una descripción.

| Código | Módulo | Clasificación | Descripción | Instrucción específica |
|---|---|---|---|---|
| E000 | Main | Error | Error desconocido. | Solicita revisión técnica; no reanudes hasta identificar la condición. |
| E001 | Main | Error | No se puede alimentar porque holder y encoder no están activos. | Verifica los indicadores de holder y encoder; si no confirman, solicita revisión técnica. |
| E002 | Main | Error | No se pueden accionar las pinzas porque la alimentación L/R no ha sido validada. | Verifica el resultado de alimentación de ambos lados habilitados; revisa el error específico si aparece. |
| E003 | Main | Error | No se puede ir a corte porque las pinzas, holder o encoder no están en el estado requerido. | Verifica los estados de las tres sujeciones en la HMI; no ordenes movimiento hasta confirmar la condición requerida. |
| E004 | Main | Error | No se puede cortar porque LM Guide no está en posición de corte. | Verifica la posición indicada del LM Guide; solicita revisión si no coincide. |
| E005 | Main | Error | No se puede descargar porque el corte no se ha completado. | Verifica la confirmación de corte; no retires material ni repitas el corte manualmente. |
| E006 | Main | Error | No se puede regresar a Home porque la pieza no se ha descargado. | Verifica la confirmación de descarga; no ordenes el regreso del eje hasta confirmar la condición. |
| E007 | Main | Error | Longitud del modelo no definida. | Verifica el modelo seleccionado y su longitud programada. |
| E010 | PLC | Comunicación | Sin respuesta de status. | Abre Conexión y prueba el puerto del PLC; si no responde, solicita revisión del enlace. |
| E011 | PLC | Error | Holder no confirmado en estado lógico activo. | Verifica la confirmación de cierre del holder en la HMI. |
| E012 | PLC | Error | Encoder no confirmado en estado lógico activo. | Verifica la confirmación de cierre del encoder en la HMI. |
| E013 | PLC | Error | Pinzas no confirmadas activas. | Verifica la confirmación de cierre de las pinzas; no intervengas mientras puedan moverse. |
| E014 | PLC | Error | Holder no confirmado liberado. | Verifica la confirmación de apertura del holder; no fuerces la sujeción. |
| E015 | PLC | Error | Encoder no confirmado liberado. | Verifica la confirmación de apertura del encoder; no fuerces la sujeción. |
| E017 | PLC | Comunicación | No hay puerto seleccionado. | Abre Conexión, selecciona el puerto del PLC y pulsa Probar. |
| E018 | PLC | Comunicación | El puerto seleccionado no es el MCU PLC. | Abre Conexión, selecciona el puerto correcto del PLC y pulsa Probar. |
| E019 | PLC | Comunicación | El puerto no respondió. | Abre Conexión y prueba el puerto del PLC; si persiste, solicita revisión. |
| E020 | PLC | Error | Fallo de holder desde el PLC físico. | Consulta el fallo del holder; solicita revisión de la sujeción y su confirmación. |
| E021 | PLC | Error | Fallo de encoder desde el PLC físico. | Consulta el fallo del encoder; solicita revisión de su sujeción y confirmación. |
| E022 | PLC | Error | Fallo de cortador desde el PLC físico. | No introduzcas las manos; solicita revisión del cortador y su condición segura. |
| E023 | PLC | Error | Fallo de pinzas desde el PLC físico. | Verifica la indicación de pinzas; solicita revisión antes de intervenir en el material. |
| E030 | Motion | Error | Alimentación ocupada. | Espera a que finalice la operación activa; si permanece ocupada, solicita revisión técnica. |
| E031 | Motion | Error | Longitud objetivo no definida. | Verifica el modelo y la longitud objetivo antes de volver a alimentar. |
| E032 | Motion | Error | Alimentadores CAN no listos. | Verifica el estado de los alimentadores; solicita revisión si no están listos. |
| E033 | Motion | Error | Timeout de láser lado R. | Verifica la presencia de material y el estado del láser derecho; no fuerces el avance. |
| E034 | Motion | Error | Timeout de láser lado L. | Verifica la presencia de material y el estado del láser izquierdo; no fuerces el avance. |
| E035 | Motion | Error | Encoder fuera de tolerancia lado R. | Verifica la medición del lado derecho y la longitud del modelo; solicita revisión si persiste. |
| E036 | Motion | Error | Encoder fuera de tolerancia lado L. | Verifica la medición del lado izquierdo y la longitud del modelo; solicita revisión si persiste. |
| E037 | Motion | Error | Fallo al aplicar compensación. | Verifica el ajuste de compensación del modelo; solicita revisión antes de repetir el movimiento. |
| E038 | Motion | Error | Fallo durante la alimentación. | Verifica el estado de alimentación y solicita revisión si no se identifica una causa específica. |
| E039 | Motion | Error | Señal de seguridad activa. | Identifica la condición de seguridad activa; no restablezcas ni reanudes hasta verificar que es seguro. |
| E040 | Motion | Comunicación | Sin respuesta de status. | Abre Conexión y prueba el puerto de Motion; si no responde, solicita revisión. |
| E041 | Motion | Comunicación | No hay puerto seleccionado. | Abre Conexión, selecciona el puerto de Motion y pulsa Probar. |
| E042 | Motion | Comunicación | El puerto seleccionado no es el MCU Motion. | Abre Conexión, selecciona el puerto correcto de Motion y pulsa Probar. |
| E043 | Motion | Comunicación | El puerto no respondió. | Abre Conexión y prueba el puerto de Motion; si persiste, solicita revisión. |
| E044 | Motion | Error | No se confirmó la posición de destino del LM Guide. | Verifica la posición indicada y el estado del drive; no retires material ni ordenes movimiento hasta validar una posición segura. |
| E045 | Motion | Error | Alimentación interrumpida. | Verifica por qué se detuvo la alimentación y que no exista una orden de paro activa. |
| E046 | Main | Error | No se confirmó el restablecimiento del sistema. | Consulta si existe un código específico activo; si no se identifica la causa, solicita revisión técnica. |
| E053 | Motion | Error | Material terminado en lado R. | Verifica el suministro derecho. Pulsa Reset y sigue la recuperación. |
| E054 | Motion | Error | Material terminado en lado L. | Verifica el suministro izquierdo. Pulsa Reset y sigue la recuperación. |
| E055 | Motion | Error | Material terminado en ambos lados. | Verifica el suministro de ambos lados. Pulsa Reset y sigue la recuperación. |
| E070 | Andon | Comunicación | No hay puerto seleccionado. | Abre Conexión, selecciona el puerto de Andon y pulsa Probar. |
| E071 | Andon | Comunicación | El puerto seleccionado no es el MCU Andon. | Abre Conexión, selecciona el puerto correcto de Andon y pulsa Probar. |
| E072 | Andon | Comunicación | El puerto no respondió. | Abre Conexión y prueba el puerto de Andon; si persiste, solicita revisión. |
| E090 | PreAlimentador | Comunicación | No hay puerto seleccionado. | Abre Conexión, selecciona el puerto del maestro PreAlimentador y pulsa Probar. |
| E091 | PreAlimentador | Comunicación | El puerto seleccionado no es el maestro PreAlimentador. | Abre Conexión, selecciona el puerto correcto del maestro y pulsa Probar. |
| E092 | PreAlimentador | Comunicación | El puerto no respondió. | Abre Conexión y prueba el puerto del maestro PreAlimentador; si persiste, solicita revisión. |
| E093 | PreAlimentador L | Error | Falla de cilindro. | Verifica el estado del cilindro izquierdo; solicita revisión antes de manipular el mecanismo. |
| E095 | PreAlimentador L | Error | Holgura no confirmada. | Verifica el suministro de material y el indicador de holgura izquierdo; solicita revisión si no se recupera. |
| E096 | PreAlimentador R | Error | Falla de cilindro. | Verifica el estado del cilindro derecho; solicita revisión antes de manipular el mecanismo. |
| E098 | PreAlimentador R | Error | Holgura no confirmada. | Verifica el suministro de material y el indicador de holgura derecho; solicita revisión si no se recupera. |
| E099 | PreAlimentador | Error | No está listo ni estabilizado. | Verifica el estado del PreAlimentador y las fallas activas; usa Iniciar únicamente cuando esté habilitado. |

## 4. Guías por etapa y rutas existentes

Esta sección fija los textos de referencia que ya existen en la recuperación convencional. Las acciones, condiciones y transiciones se conservan tal como están implementadas; no convertir estas filas en una nueva máquina de estados.

| Etapa | Guía / acción visible | Condición |
|---|---|---|
| Error activo | Instrucción específica del catálogo; Reset | Solo según habilitación vigente. |
| Reset bloqueado | Código específico confirmado o E046 | No avanzar. |
| Revisión de piezas | Confirma que las piezas están bien. / Confirmar | Reset exitoso y condiciones vigentes. |
| Sensores | Retira el material hasta apagar ambos sensores láser. Cuando digan Apagado, pulsa Confirmar. | Confirmar solo cuando los sensores y demás condiciones vigentes lo permitan; no presentar como autorización de intervención física. |
| Zona segura en curso | Zona segura: moviendo el eje. Espera. | Conservar interlocks y confirmaciones existentes. |
| Zona segura lista | Zona segura lista. Retira el material hasta Apagado y pulsa Confirmar. | Solo después de confirmaciones existentes; Park por sí solo no certifica seguridad para introducir manos. |
| Purga en curso | NO INTRODUCIR MANOS / Proceso de limpieza en curso, espera… / Cancelar | No habilitar acciones incompatibles. |
| Purga fallida | Instrucción Retry del apartado siguiente / Retry / Cancelar | Mostrar el código real de la nueva falla si existe. |
| Purga cancelada | Purga cancelada. Puedes Confirmar para reintentar. | Retorno a sensores según lógica vigente. |
| Listo | Purga terminada. Vuelve a la pantalla principal y pulsa Reanudar. Esta pieza no suma al contador. / Volver a la pantalla principal | Conservar conteo y estado del lote. |

Skip conserva sus restricciones y registro de empleado. No habilitarlo con error activo ni movimiento de purga. Cerrar una ventana no limpia ni cancela una recuperación pendiente.

### Textos Retry existentes

| Código de falla durante purga | Instrucción |
|---|---|
| E020 | Revisa el holder y pulsa Retry. |
| E021 | Corrige el encoder y pulsa Retry. |
| E022 | Revisa el cortador y pulsa Retry. |
| E023 | Revisa las pinzas y pulsa Retry. |
| E033 | Revisa el sensor láser derecho y pulsa Retry. |
| E034 | Revisa el sensor láser izquierdo y pulsa Retry. |
| E035 | Revisa el encoder derecho y pulsa Retry. |
| E036 | Revisa el encoder izquierdo y pulsa Retry. |
| E044 | Revisa el LM Guide y pulsa Retry. |
| Sin código específico aplicable | Revisa el material y pulsa Retry. |

Estos textos existentes requieren las mismas condiciones de seguridad que cualquier intervención; no son autorización para acceder a mecanismos energizados. Si un texto o la ruta resulta incompatible con una condición real, reportar el conflicto al propietario en vez de inventar una acción o modificar la secuencia.

## 5. Seguridad y pendientes

Las instrucciones específicas de la tabla son las propuestas del Excel entregado al propietario, no una validación de seguridad ni procedimientos de mantenimiento. No deben sustituir interlocks, paro seguro, aislamiento de energía cuando corresponda ni confirmaciones de posición. En particular, E003, E039, E044, E095 y E098 requieren verificar que su texto preserve el significado exacto de la detección. Si la recuperación convencional requiere mover el eje o retirar material en condiciones no confirmadas, no implementar una nueva ruta por cuenta propia; documentar el caso y solicitar aprobación antes de habilitar cambios de comportamiento. La revisión visual no equivale a aprobación de cambios de lógica.

## 6. Fuente y mantenimiento

Fuente de ejecución: `Class/error_codes.py`; especificación humana: este archivo. No interpretar Markdown en tiempo de ejecución. No mantener catálogos alternativos en JavaScript, HTML o `app.py`. Usar datos estructurados del catálogo para la HMI, sin parsear frases de diagnóstico como fuente primaria. Los identificadores y mensajes técnicos del protocolo se conservan internamente. Validar correspondencia automática del catálogo ejecutable con esta tabla.

Base: Excel `DMAT_TM_Catalogo_errores_instrucciones.xlsx`; catálogo GitHub `Class/error_codes.py` SHA de archivo `11baf379270bdbd305b180932f5f09a4a4b302c0`; lógica `app.py` y `static/js/main.js` revisadas en la conversación. No inventar códigos ausentes (incluidos huecos numéricos).
