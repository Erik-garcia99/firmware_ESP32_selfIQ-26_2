"""Exploración E2, NO clasificador de ocupación ni umbrales finales.
Ejemplo: python tests/evaluate_calibration_photos.py /ruta/imagenes_calibracion.zip
Dependencia PC: pip install pillow. Orden de fotos conocido por hora del 09-11-2026.
Por defecto usa la ROI activa suministrada por el usuario el 09/11/2026.
La geometría histórica aplicada al nodo no puede inferirse del JSON actual.
"""
import argparse
import io
import itertools
import math
import json
import statistics
import zipfile
from PIL import Image


def descriptor(im, bbox, grid):
    x0,y0,x1,y1=bbox
    pix=im.load()
    values=[]
    for cy in range(grid):
        a=y0+cy*(y1-y0)//grid
        b=y0+(cy+1)*(y1-y0)//grid
        for cx in range(grid):
            c=x0+cx*(x1-x0)//grid
            d=x0+(cx+1)*(x1-x0)//grid
            samples=[pix[x,y] for y in range(a,b) for x in range(c,d)]
            if not samples: raise ValueError('ROI demasiado pequena')
            values.append((sum(samples)+len(samples)//2)//len(samples))
    return values


def mae(a,b):
    return sum(abs(x-y) for x,y in zip(a,b))/len(a)


def describe(vals):
    return {'n':len(vals),'min':round(min(vals),2),'mean':round(statistics.mean(vals),2),
            'max':round(max(vals),2)} if vals else None


ACTIVE_ROI = (0.079044, 0.518627, 0.599265, 0.477941)
INACTIVE_ROI = (0.032799, 0.052827, 0.303859, 0.472103)
ACTIVE_ROI_CREATED_AT_UTC = '2026-11-09T21:38:58.364532+00:00'


def roi_rect(roi, width=320, height=240):
    """Misma conversión que cal_roi_to_rect: piso origen, techo extremo."""
    x, y, w, h = roi
    if not (0 <= x < 1 and 0 <= y < 1 and w > 0 and h > 0
            and x + w <= 1 and y + h <= 1):
        raise ValueError('ROI normalizada invalida')
    # Las coordenadas SQL tienen seis decimales: convertir a millonésimas.
    xi, yi, wi, hi = (round(v * 1_000_000) for v in roi)
    return (xi * width // 1_000_000, yi * height // 1_000_000,
            ((xi + wi) * width + 999_999) // 1_000_000,
            ((yi + hi) * height + 999_999) // 1_000_000)


def run(path, bbox):
    with zipfile.ZipFile(path) as z:
        names=sorted(n for n in z.namelist() if n.lower().endswith(('.jpeg','.jpg')))
        frames=[Image.open(io.BytesIO(z.read(n))).convert('L') for n in names]
    if len(frames)!=25:
        raise ValueError('Etiquetado provisional SOLO para las 25 capturas conocidas, llegaron %d' % len(frames))
    if any(im.size!=(320,240) for im in frames):
        raise ValueError('Resolucion inesperada; revisar datos de origen')
    # Etiquetas inferidas visualmente; confirmar con el operador del montaje.
    groups={'vacia':list(range(0,7)),'cinco':list(range(7,16)),
            'cuatro':list(range(16,19)),'tres':list(range(19,22)),
            'dos':list(range(22,25))}
    output={'fuente':path,'bbox_px':bbox,'nota_temporal':'ROI activa creada 2026-11-09 21:38:58 UTC; imágenes anteriores son evaluación retrospectiva, no evidencia de ROI aplicada','grupos_provisionales':{k:len(v) for k,v in groups.items()},'comparacion':{}}
    for grid in (8,16):
        ds=[descriptor(im,bbox,grid) for im in frames]
        def between(a,b):return [mae(ds[i],ds[j]) for i in groups[a] for j in groups[b]]
        def within(a):return [mae(ds[i],ds[j]) for i,j in itertools.combinations(groups[a],2)]
        output['comparacion'][str(grid)]={
          'vacia_interna':describe(within('vacia')),
          'cinco_interna':describe(within('cinco')),
          'vacia_vs_cinco':describe(between('vacia','cinco')),
          'vacia_vs_cuatro':describe(between('vacia','cuatro')),
          'vacia_vs_tres':describe(between('vacia','tres')),
          'vacia_vs_dos':describe(between('vacia','dos')),
        }
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('zip',help='imagenes_calibracion.zip exportado')
    parser.add_argument('--bbox',type=int,nargs=4,default=None,metavar=('X0','Y0','X1','Y1'),
                        help='Rectángulo de pixeles opcional; si se omite utiliza la ROI activa real')
    args=parser.parse_args()
    bbox = tuple(args.bbox) if args.bbox is not None else roi_rect(ACTIVE_ROI)
    if not (0<=bbox[0]<bbox[2]<=320 and 0<=bbox[1]<bbox[3]<=240):
        parser.error('bbox fuera de 320x240')
    print(json.dumps(run(args.zip,bbox),indent=2,ensure_ascii=False))
