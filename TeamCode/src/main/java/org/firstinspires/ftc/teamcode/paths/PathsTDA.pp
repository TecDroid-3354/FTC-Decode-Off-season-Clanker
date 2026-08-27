{
  "startPoint": {
    "x": 21.81086519114688,
    "y": 122.7364185110664,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-qyms922sj39",
      "name": "Path 1",
      "endPoint": {
        "x": 49.04627766599597,
        "y": 104.66800804828975,
        "heading": "linear",
        "startDeg": 144,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#9989DC",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mt4hai7l-mphomr",
      "name": "Path 2",
      "endPoint": {
        "x": 48.83299798792757,
        "y": 84.06639839034203,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#C5A979",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mt4hb1f6-8bc5ve",
      "name": "Path 3",
      "endPoint": {
        "x": 17.35211267605634,
        "y": 84.37223340040241,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#AC66D8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mt4hbiny-rv5q5p",
      "name": "Path 4",
      "endPoint": {
        "x": 38.8712273641851,
        "y": 110.71428571428571,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#665A7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-qyms922sj39"
    },
    {
      "kind": "path",
      "lineId": "mt4hai7l-mphomr"
    },
    {
      "kind": "path",
      "lineId": "mt4hb1f6-8bc5ve"
    },
    {
      "kind": "path",
      "lineId": "mt4hbiny-rv5q5p"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-08-22T14:35:24.543Z"
}