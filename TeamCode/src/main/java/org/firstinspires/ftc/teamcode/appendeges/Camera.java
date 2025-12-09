package org.firstinspires.ftc.teamcode.appendeges;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Camera {
  private HuskyLens camq;

  public Camera(HardwareMap hardwareMap) {
    camq = hardwareMap.get(HuskyLens.class, "camq");
  }

  public class Update implements Action {
    private boolean initialzed = false;
    private int tagx = -1;
    private int tagy = -1;
    private int tagw = -1;
    private int tagh = -1;
    private int tagid = -1;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      if (!initialzed) {
        camq.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        initialzed = camq.knock();

      }
      HuskyLens.Block[] blocks = camq.blocks();
      packet.put("Block count", blocks.length);
      if (blocks.length > 0) {
        for (int i = 0; i < blocks.length; i++) {
          packet.put("Block", blocks[i].toString());
        }
        tagx = blocks[0].x;
        tagy = blocks[0].y;
        tagw = blocks[0].width;
        tagh = blocks[0].height;
        tagid = blocks[0].id;
      } else {
        tagx = -1;
        tagy = -1;
        tagw = -1;
        tagh = -1;
        tagid = -1;
      }
      return camq.knock();
    }
  }
}
