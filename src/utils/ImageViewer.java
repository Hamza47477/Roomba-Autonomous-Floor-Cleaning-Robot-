package utils;

import java.awt.*;
import javax.swing.*;

public class ImageViewer
{
    private static JFrame editorFrame;
    private static ImageIcon imageIcon = new ImageIcon();

    public static void display(Image image)
    {
        display(image, false);
    }

    public static void display(Image image, boolean adjustSize)
    {
        imageIcon.setImage(image);

        if(adjustSize)
        editorFrame.setSize(new Dimension(image.getWidth(null), image.getHeight(null)));

        if(editorFrame != null)
        editorFrame.repaint();
    }

    public static void open(int width, int height, String title)
    {
        SwingUtilities.invokeLater(() ->
        {
            editorFrame = new JFrame(title);
            editorFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

            editorFrame.setSize(width, height);
            editorFrame.setPreferredSize(new Dimension(width, height));

            JLabel jLabel = new JLabel();
            jLabel.setIcon(imageIcon);
            editorFrame.getContentPane().add(jLabel, BorderLayout.CENTER);

            editorFrame.pack();
            editorFrame.setLocationRelativeTo(null);
            editorFrame.setVisible(true);
        });
    }
}
