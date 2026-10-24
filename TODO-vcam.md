# hgvcam — pendências

## Implementado em 2026-10-23, testado (funcionou)

A falha ao trocar a resolução no Multiview era do próprio Multiview
(`v_mat[0]` com o tamanho antigo, em `v4l2misc/camview.cpp`).

- **Player não recarrega módulo em uso.** Em `prepare_module`
  (`hgvcam_player.c`), se a versão carregada difere do `.ko` e
  `/sys/module/hgvcam/refcnt` > 0, só avisa e segue com o módulo carregado.
- **Várias resoluções na câmera (o player redimensiona).** `S_FMT` na câmera
  (fora de streaming) grava o formato e avisa o player por evento V4L2 no nó
  produtor; o player refaz `S_FMT` no produtor (fecha/reabre o nó) e
  redimensiona a partir do vídeo original. Frames de tamanho errado são
  descartados pelo módulo. Proporção diferente: barras pretas.
- **Lista de fps.** 60, 30, 25, 24 e 15 fps, mais o fps do arquivo. O player
  gera frames na taxa pedida escolhendo o frame do vídeo pelo timestamp (pts).
- **Pause congela a imagem** (repete o último frame; despausar continua do
  mesmo ponto), no player.
- **Controles de imagem** padrão (brilho, contraste, saturação, flips): o
  módulo guarda, o player aplica — brilho/contraste/saturação por tabelas
  próprias (LUT) sobre o YUYV, pois o swscale ignora esses ajustes em YUV→YUV.
- **Zoom digital com pan/tilt** (zoom 100–1000 = 1×–10×; pan/tilt −100 a 100).

## Implementado em 2026-10-24, testado (funcionou)

Faltam testes com outros vídeos (resoluções, fps, proporções e codecs
diferentes), a fazer pelo usuário.

- **Câmeras virtuais dinâmicas.** O módulo carregado cria só
  `/dev/hgvcam`; cada player cria a sua câmera com o ioctl
  `HGVCAM_ADD` e ela some quando o player sai (`HGVCAM_REMOVE`, ou o
  fechamento do dispositivo de controle — vale também se o player travar ou
  for morto). Nomes numerados "Hitgalut Virtual Camera N" / "Hitgalut Virtual Camera
  producer N". Limite `HGVCAM_MAX_CAMERAS` = 15 em `hgvcam.c`. Cada
  câmera é liberada no último `close()` (callback `release` do
  `v4l2_device`); um aplicativo que ainda a tenha aberta recebe `ENODEV`.
  - Regra udev `70-hgvcam.rules` (grupo `video` + `uaccess` para o
    usuário logado): o player a instala no mesmo passo com sudo que carrega o
    módulo; depois disso não é preciso senha enquanto o módulo estiver
    carregado.
  - Opção `--device` removida: cada player usa só a câmera que criou.
  - Parâmetros `capture_nr`/`output_nr` do módulo removidos (os números
    `/dev/videoN` são automáticos e o player os imprime).
  - Descartado: vários aplicativos na mesma câmera (fan-out).
- **Produtor em uso só aceita consultas.** Enquanto um player é dono da fila,
  outros programas que abram o nó produtor podem consultar, mas
  `S_FMT`/`S_PARM` devolvem `EBUSY`. "File exists" do `insmod` conta como
  sucesso (dois players iniciados juntos com o módulo descarregado).
- **Player suspende o trabalho sem captura.** O módulo informa se há
  aplicativo capturando (`HGVCAM_CAMERA_STREAMING` em
  `struct hgvcam_camera`, pelo evento e por `HGVCAM_G_CAMERA`). Sem
  captura o player não decodifica nem converte, e o vídeo fica pausado;
  quando a captura volta, continua de onde parou.
- **Resolução nativa na lista.** O tamanho do vídeo (aspecto de pixel
  aplicado, reduzido para caber em 1920×1080) entra na lista junto com as
  cinco padrão, e a câmera começa nele e no fps do vídeo. Ele é informado no
  próprio `HGVCAM_ADD` (e não num ioctl separado no produtor, como estava
  previsto): a câmera já nasce com ele.
- **Mais formatos na câmera:** YUYV, MJPEG (cada quadro codificado como JPEG
  pelo FFmpeg, qualidade `JPEG_QSCALE` = 3), BGGR (`SBGGR8`) e GBRG
  (`SGBRG8`), montados pelo player a partir de RGB. Todos com as mesmas
  resoluções e fps. MPEG descartado (fluxo multiplexado que aplicativos de
  câmera não aceitam). O produtor usa `fileio_write_immediately`: cada
  `write()` é um quadro, então quadros MJPEG de qualquer tamanho passam.
- **Controle black no player**, como os demais; o módulo não altera mais os
  quadros.

## Observações da implementação

- Todos os formatos passam pela mesma composição em YUYV (zoom, barras pretas,
  flips, LUT, black) e são convertidos no fim: MJPEG via YUVJ422P, Bayer via
  RGB24.
- README atualizado.
- GUI removida em 2026-10-24 (`hgvcam_gui.c`, binário, alvos `gui`/`app`
  do Makefile e seção do README): o projeto não terá interface gráfica.
