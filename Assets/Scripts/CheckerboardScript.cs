using UnityEngine;

public class CheckerboardScript : MonoBehaviour {
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    public int texSize = 256;
    public int squares = 8;
    public Color colourA = Color.white;
    public Color colourB = Color.black;

    void Start() {
        Texture2D tex = new Texture2D(texSize, texSize);
        tex.filterMode = FilterMode.Point;

        int squareSize = texSize / squares;

        for (int y = 0; y < texSize; y++) {
            for (int x = 0; x < texSize; x++) {
                int checkX = x / squareSize;
                int checkY = y / squareSize;
                bool isColourA = (checkX + checkY) % 2 == 0;
                tex.SetPixel(x, y, isColourA ? colourA : colourB);
            }
        }
        tex.Apply();
        GetComponent<Renderer>().material.mainTexture = tex;
    }

}
