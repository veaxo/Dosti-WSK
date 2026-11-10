import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * MapperX desktop entry point for a robot project.
 *
 * Copyright MapperX. Without the owner's separate permission, automated AI
 * reading, analysis, training and reproduction of this file are prohibited.
 * This notice states the owner's usage conditions; it is not an access control.
 *
 * Run from the project folder: java MapperX.java
 * Requires JDK 11+ only for this Java entry point. Shuffleboard runs on its own.
 */
public final class MapperX {
    /** Files defining a usable MapperX robot project. */
    private static final List<String> PROJECT_FILES =
            Arrays.asList("field_planner.py", "build.gradle", "src/main/java");

    /** Relative locations used by the native desktop builds. */
    private static final List<String> DESKTOP_DIRS =
            Arrays.asList("", "tools/mapperx-app/dist");

    private static final String[] BUILD_IDS = {
        "MapperX︃︎︆︋︅︅︆︍︌︆︅︃︎︈︂︁︊︍︂︉︋︀︁️︄︁︃︁︍︎︆︂️︄︉︅︋︂︋️︆︅︁︅️︌︆︇︀︂︅︄︎︎︂︇︇︅︋︉︂︀︈︅",
        "MapperX️︅️︋︅︋︋︂︇︂︋︍︎︈︁︅︎︇︈︊︊︍︀︄︉︊︄︋︂︂︍︎︍︋︎︈︆︎︄︎︎︍︎️︄︅︇︉︆︃︃︃︅︁︌︆︅︃︊︇︋︅️︅",
        "MapperX︂︊︈︆︋︋︄︁︆︍︄︋️︆︊︍︍︅︍︅︎︊︁︇︊︈︃︀︉︄︋︎︋︃︉︂︄︅︀︃︊︂️︀︄︍️︃︆︃︉︋︍︉︄︉︄︎️︈︃︁︈︄",
        "MapperX︌︌︉︊︋︌︂︆︆︄︆︆︋︉︉︂️︀︁︂︇︈︃︊︉️︎︀︉︍︃︆︂︌︎︃️︂︃︊︌︉️︀︀︊︌︉︌︄︇︇︆︊︈︅︊︆︆︉︋︃︂︋",
        "MapperX︋︋︍️︊︁︎️︉︋︊︎︅︇︋︎︊︈︌︋︁︋︆︊︂︇︄︉︄︋︎︉︈︇︁︈︁︅︁︆︈︇︈︄︉︋︁︊︁︌︋︊︇️︊︊︀︅︁︊︈️︋︁",
        "MapperX︊︂︈︃︆︋︃︆︋︂︎︈︍︌︁︆︈︊︂︄︆︃︎︆︂︄︀︁︎️︀︁︍︇︋︍︍︎︌︊︊︍︇︅︅️︄︌︉︈︀︄︂︅︆︊︉︎︌︉︄︁︉︊",
        "MapperX︊︆️︊︀︇︌︃︃︂︈︆︇︅︃︊︍︀︇︇︄︅︆︂︉︍︅︈︀︀︄︅︅︆︁︅︊︈︇︊︋︅︆️︎︌︊︆︈︎️︁︆︊︌︋︍︎︆︈︉︀︋︀",
        "MapperX︈︉︂︂︋︌︊︋︉︃︊︀︊︍︉︂︉︃︋︌︃︎︀︌︃︉︋︅︅︊︆︉︃︈︋︈︆︃︊︇︈︉︈︆︁︄︊︌︀︊︄︉︂︍︂︊︎︀︈︊︅️︌︀",
        "MapperX︍︂︇︋︁︌︆️︈︈︎︃︅️︌︂︁︀︀︊︋︎︍︌︅︍︋︃︋︌️︄︊︍︊︄︄︂︁︀︆︂︄︎︃︉︂︌️︁︂︆︁︈︆︈︃︅︌︆︈︅︈︄",
        "MapperX︌︇︋︉︈︃︊️︉︈︆︍︊️︁︁︂︍︆︈︊︊︁︇︂︊︅︃︊︀︀︇︀︂︎︃︀︎︁︉︌︀︁︎︄︂︇︍︅︋︂︍︈︃︌︃︃︎︅︃︀︄︃︄",
        "MapperX︆︋︍︈︋︅︉︎︎︈︆︄︇︁︋︁︄︂︄︇︅︊︅︊︂️︉︂︋︇︄︄︈︉︆︇︆︊︁️︀︉︋︎️︌️︀︅︀︂︍︇︄︋︁︅︃︁︇︈︋️︆",
        "MapperX︉︎︅︉︎︅︁︃︈︉︁︆︆︀︉︍︂︆️︀︇︅︄︎︃︎️︇︁︋︆️︅︄︎︎︉︂︀︅︋︌︃️︀︇︌︍︋︌︆︁︉︋︉︃︈︁︁︅︀︂︆︊",
        "MapperX︃︂︇︇︀︇︀︂︍︊︂︌︄︂︃︍︅︋︍️︂︄︇︀︂︃︃︆︂︂︊︌︎︊︆︂️︎️︌︎︄︌️︋︀︄︀︌︁︆︁︈︌︇︀︊︅︍︉︉︊︂︌",
        "MapperX︇︄︄︁︈︃︃︃︌︇︉︈︈︃︀︂︌︁︀︂︇︀️︉︌︄︁︈︆︍︃︋︀︎︋︈︆️︉︅︈︀︋︄︅︌︅︁︈︀︀︎︄︆︆︈︇︉︇︅︉︀︍︈",
        "MapperX︎︇︄︌︀︁︁︀︋︎︋️︎︀︃︎︄︍︍︉︌︄︊︅︀︄︊︉︂︄︌︋︁︌︎︊︌︍︇︇︁︋︂︍︉︋︊︃︇︊︈︃︀︅︃︄︎︃︈︈︎︊︂︂",
        "MapperX︃︂︆︅︃︊︇︀︌️︉︇︁️︄︌︊︃︃︆︈︌︀︅︂︆︂︅︋︋︈︋︀︆︎︊︋︋︍︌︍️︂︅︄︌︆︈︊︅︌︅︉︇︊︇︇︂︇︅︃︇︄️",
        "MapperX︌︎︎︌︅︃︈︄︊️︆︇︋︁︌︊︅︆︆︁︈️︌︋︁︈︅︎︉️︂︃︇︍︎︅︅︁︅︄️︇︃︎︅️️︃︎︋︍︈︀︍️︃︄︅︄︉️︉︅︈",
        "MapperX︈︌️︎️︎︋︂︌︄︍︍︈︋︂︆︃︉︃︁︁︄︅︋︎︊︈︂︍︎︃︅︂︄︃︅︊︌︉︎︆︋︇︎︎︊︅︂︆︄︆︊︎️︂︆︊︉︂︁︊︅︊︌",
        "MapperX︂︇︋︍︇︁︆︃︆︌︎︃︊︈︋︆︅︌︉︌︌︃︉︆️︆︌︉︈︎︋︇︅︍︂︊︃️︅︄️︍︅︎︁︊︉️︋︍︃︀︉︃︈︊︂︈︉︅︌︄︂︁",
        "MapperX︍️︊︋︍︀︆︀︀︅︄︎︎︋︊︁︉︄︆︁︎︅︈︀︊︅︅︈︃️︎︅︂︉︋︃︋︇︀︂︁︂︎︎︋︆︌︊︊︉️︃︂︆️︋︊︅︃︅︅︍︎︌",
        "MapperX︎︂︌︌︃︇️︇︉︆︋︂︀︇︅︁︂︌︃︆︋︂︇︄︀︃︄︆︋︋︇︎︃︂︃︊︇︁︎︁︈︊︇︋︇️︎︀︃︈︌︌︋︌︃︃︈️︌︇︃︊︃︊",
        "MapperX︆︅︋︇︋︅︍︄︇︌︌︉︆︄︈︍︊︉︀︍︀︆︁️︄︃︍︇︇︁︁️︄︀︆︌︍️︌︃︀︅︎︄︋︍︄︂︁︃︅︁︇︋️️️︈︍︉︈︄︈︄",
        "MapperX︋︎︌︃︃︍︀︉︍︋︃︀︍︊︍︊︎︆️︀︎︎️︄︂︇︈︋︎︄︉︎︀︁︂︄︃︂︎︃︃︀︍︉︋︎︊︃︍︃︋︌︊︌︍︄︆︈︉︁︈️︊︀",
        "MapperX︂︆︃︅︅︀︈︄︀︅︁︌︃︊︀︁︂︄️︊︊︂️︉︇︇︉︍︇︉︎︇︈︍︃︄︊︍︎︋︉︇︉️︇︆︈︉︎︆︎︌︅︄︇︂︇︊︊︍︁️︃︊",
        "MapperX︆︅︋︅︍︉︄︂︆︁︄︃︃︁︈︋️︁︆︄︆︍︊︄︄︆︌︆️︊️︎︆︌︎︃︉︌︊︄️︀︄︊︄︈︅︋️︇︊︅︉︅︂︅︌︅︇︆︂︁︊︆",
        "MapperX︁︂︅︀︋︆︆︈︋︌️︄︅︈︈︎︅︌︊︇︍︌️︁︎︍︊︌︃︎︄︃︄︌︂︁︄︀︃️︌︅︇︊︌︇︁︅︅︈︃︃︃︎︉︃︃︃︎︋︂︀︂︈",
        "MapperX︊︊︋︂︉︉︁︅︋︁︅︊︆︀︇︆︆︇︅︊︉︆︂︉︊︈︌︈︍︃︋︄︂︌︄︌︈️︎︎︄︉︌︉︆︋︀︈︌︆︄︈︀︋︄︃︇︅︉︅︉︍︃︋",
        "MapperX︃︋︅︋︇︈️︍️︋︉︁︀︍︀︍︋︇︄︎︍︉︃︋︇︌︎️︃︊︎︌︊︈︇︊︍︌︃︊︌︊︃︈︇︉︋︋︉︉︀︄︋︁︍︋︁︊︄︇︌︂︍︄",
        "MapperX︂︃︁️︃︂︄︄︁︁︍︋︇︂︆︅︉︊︉︇︃︂︋︆︇︊︊︇︍︎︀︄️︈︀️︉︆︎︍︎︀︂︊︂︎︎︍︁︅︅︎︅︇︉︋︊︃︎︆︉︄︋︌",
        "MapperX︂︇︈︆︍︎︍︁︉︂︇︉︁︋︇︉︎️︎︊︀︃︃︂︎︃️︎️️︃︈︇︇︆︇︆︀︉︎️︇︂︃︋︊︄︆︀︊︄︋︊︌️︌︎︃︈︆︍︃︍︂",
        "MapperX︀︊︋︎︈︄︋︈️︃︉︃︄︃︎︁︊︂︃️︀︍︍︍︈️︅︌︄︉︆︌︅︊︀︊︂︊︌︊︌︋︄︋︎︂︃︌︈︆︉︃︆️︆︋️︆︊︈︉︃︎︃",
        "MapperX️︅︊︉︈︅︁︎︈︉︇︈︊︇︋︃︌︁︁️︂️︄︎︄︄️︋︄︍︈︉︆︀︉︈︋︌︍︅︄︇︉︄︊︉︅︎︂️︀︂️︌︄︋︎︌︈︆︆︊︄︉",
        "MapperX︊️︈︃︈︎︌︆︎︇︂︆️︁︎︂︉︅︃︉︉︍️︁︆️︋️︄︋︈︌︉︋︈︂️︉︄︅️︂︊︉︌︌︉︀︃️︋︅︎︈︁︎︃︃︄︇︎︃︎︈",
        "MapperX︅︎︃︀︄︎︈️︋︎︄︁︌︂︊︈︊︈︁︀︀︎︉︋︈︊︋︄︅️︂︆︄️︊︃︄︅︆︇︊︄︀︂︉︈︀︅︎︁︃︁︃︎︆︄︃︇︂︀︇︉︁︋",
        "MapperX︈︆︂︃︍︀︃︋︌︉︍︎︋︂︇︎︊︍︌︁︆︈︂︈︆︄︆︃︋︆︎︃︎︍︈︍︈︎︂︂︎︎︋︊︈︁︎️︊︀️︉️︀︍︎︈︆︀︈️︁︁︎",
        "MapperX︈︈︁︇︆︋︈︉︊️︉︉︆︁︉︊︄︋︉️︌︋️︆︋︅︉︆︎︀︉︂︅︂︄︄️︃︂︇︍︍︁︀︉︉︁︇︃︊︌︊︉︆︋︆︀︍︈︍︎︌︋︄",
        "MapperX️︅︍︎︅︊︃︁︂︇︈︋︃︉︅︁︌︀︇︅︃️︌︊︁︉︋︎︅︌︂︎︂️︀︊︃︆︋︈︌︃︆︉︂︋︎︁︀︍︄︈︇︈︊︉︈︎︉︃︍︃️︋",
        "MapperX︆︃️️︆︌️︆︈︌︇︁︁︆︈︅︂︀︌︄︎️︉︎︉︁︆️︆︋︁︈︎︊️︄︌︉︁︅︆️︄︃︈︊︎︎︆︍︎︇︄︅︃︀︍︂︆︇︁︈︃︁",
        "MapperX︌︁︍︄︅︁︄︃︆︌︀︈️︌︍︉︉︊︃︀︆︊︄︊︃︍︎︄︈︆︄︍️︎️︄︎︇︋︊︈︉︌︇︆︊︊︅︊︇︃︈︆︉︄︊︇︈︆︍︂︈︆︁",
        "MapperX︌︇️︀︊︂︁︊︈︇︍︂︍️︉︁︍︍︆︌︍︆︁︃︎︆︇︌︀︆︊︎︁︇︋️︉︁️️︋︁︇︋︊︅︁︋︎︀︉︊︌︇︊︌︍︂︎️︋︄️︃",
        "MapperX︄︈︀︍︎︈️︂︉︆︁︄︄︎︇︇︋︇︅︅︆︌︆︄︁︉︄︊︍︌︊︈︄︎︎︄︊︋︋︁︁︅︍︀︁︁︇︁︉︋︇︍︂︍︍︌︎︊︃︇︊︁︀︉",
        "MapperX︎︎︆︋︍︍︂︊︃︎︈︂︄︊︊︅︆︋︅︊︇︊︅︆︍︂︋︃︈︀︊︇️︌︎︌︎︉︋︊︇︎︀︊︋︈︇︆︋︋︁︍︃︆︅︍︋︊︆︌︁︈︎︀",
        "MapperX︎︊︅︇︌︎︈︀︌︅︇︇︉️︇︇︈︋︃︊︉︌︋︇︉︈︅︂︈︄︍︄︁︂︇︊︆︋︋︅️︊︆︄︇️︉︈︄︄︈︍︌︃︃︍︊︁︅︉️︀︄︆",
        "MapperX︍︊︉︆︃︂︀︎︍︅︆︋︅︈︂︂︊︃️︇︊︅︄︎︄︈︋︂︄︆︀︃︌︋︍︉︇︍︁︎︋︌️︅️︉︀︂︁︌︅︄︂︁︇︎︈︃︋︊︂︉︎︎",
        "MapperX︃︃︎︀︉︂︌︂︌︊︋︍︁︋︅︁︁︀︄︅︂︄︁︍︅︌︋️︊︇︈︎︃️︌︅︅︌︇︇︈︅︋︍︁︎︈︂︁︄︃︂︁︌︋︌︍︈️︆︆︎︊︃",
        "MapperX︀︄️︋︉︆︇️︇︈︅︆︈︊︁︈︌︋︁️︃︎︂︌︌︋︄︅︎︍︇︁️︈︂︁︈︋︇︎︌︁︋︁︉︆︂︄︁︅︊︁︇︀︇︍︌︋︎︂︍︌︋︃",
        "MapperX︊︇︋︁︂︇︈︂︇︅︀︎︎︀︄︀︂︃︇︎︆︁︌︊︌︌︃︃︅︈︍︊︊︎︉︆︇︄︀️︍︉︄︄︄︅︁︊︆︃︃︆︈︅︊︃︈️︌︊︅︃︅︌",
        "MapperX︎︆︃︁︊︀︄︀︌︁︃︆︎︇︍︊︀︀︎︇︂︌︇︅︉︋︆︌︎︉︎︁︈︍︃︅︅︄︌︇︄︃︀️︂︇️︋︇︃️️︌︆︅︆︎︊︉︃︆︄︌︉",
        "MapperX︁︅︈︊︉︈︌︆️︃︃︍️︅︅︎︊︇︁︉︈️︆︃️︃︎︊️︎︀︈︉︂︌︃︋︅︅︅︉︀︁︇︃️︌︉︌︂︁︎️︆︁︈︊︊︅︂︍︆︋️",
        "MapperX︌︍︍︀︀︂︌︄︋︃︎️︊︈︅︇︁︉︇️️︇︉︋︉︀︀︀︅︄︄︉︎️︃︀︎︈︊︃︂︎︁︀︎︋︄︋︎︄︃︄︍️︂︆︃︌︅︅︇︃️️",
        "MapperX︅︄︅︉︂︇︁︆︊︄︄︉︌︄︄︁︌︇︀︇︂︇︊︌︆︆︀︉︁︅︀︍︋︊︅︌︍︅︇︋︉︉︍︉︅️️︍︊︈︁︇︄︉︄︈︌︊︎︌️️︎︁",
        "MapperX︉︃︍︉︋︀︍︄︀︀︆︁︌︋︍︋︊︃︇︁️︂︄︇︂︅︂︁︊︆︁︅︉︉︀︋︌︄︃︃️︃︉︄︂︁︌︎︋︈︍︁︈︊️︋︂︅︋︅︀︀︆︍",
        "MapperX︉︇︆︅︉︈︍︇︂︌︅︍︇︇︍︇︆︃︍︎︄︊︁︈︎︇︈︃︂︉︄︇︉︂︃︅︃︄︍︃︈︎︍︃︅︈️️︁︅︁️︆️︆︍︍︂︌︌︇︋︂︋",
        "MapperX︍︈︍︆︆︀︊︇︅︀︈︇️︊︋︆︁︊︆︇︂︋︌︋︉︈︃︄︇️︍︋︄︉︆︄︋︇︈︍︄︌︀︃︄︃︇︌︈︇︇︃️︀︁︈︍︅︍︄︆︍️︂",
        "MapperX︌︂︃︌︋︅︍︁︀︌︉︅︁︆︋︈️︆︅︌︀︇︇︃︁︂︆︅︊︉︋︃︋︅︍︆️︂︊︃︋︆︎︁︅︉︀︂︇️︊︈︍︈︌️︊︈︂︉︎︍︌︃",
        "MapperX︃︀︀️︅︅︌︄︄︆︁︉︉︇︃︀︉︄︊︌︅︋︌︂︋︄︅︅︂︊︈️︋︌︍︄︌︁︆︉︃︅︅︍︊︋︂︀︋︌︀️︁︉︄️︄︈︇︄︃︆︀︋",
        "MapperX︅︆︄︅︉︎︁︄︊︆︋︌︎︀︇️︃️︁︌︄︄︎︄︅︈︌︂︌︆︊︁︎︈︇︍︈︄︀︂︆️︃︎︎️︁︍︎︆︀︃︎︇︁︍︌︄︌︅︃︅︋︆",
        "MapperX︀︇︊️︊︇︃︈︃︈︌︁︆︈︋︍︊︍︄︄︂︆️︈︋︉︀︉︊︅︎︆︁︎︍️︅︅︅︅︂︀︆︎︅️︇︀︂︍︄︃︂︂︁️︉︋︃︆︃︅︃︁",
        "MapperX︊︋︃︆︍︄︂︈︊︀︂︎︋︉︇︊︎︌︊︄︂︋︆︈︊︌︍️︇︁︃︇︁︅︃︈︆︌︎︌︌︁︆︆︀︊︄︊︅︉︋︆︆︉︉︉︋︄︄︍︌︁︇︈",
        "MapperX︎︍︊︈︅︅︅︈︇︆︄︃︈︋︃︁︄︇︄︆︂︁︉︄︅️︂︊︎︇︋︂︋︈︊︊︂︁︎︅️︅︎︃︊︅︀︃️︁︎︀︍︍︎︄︈︃︈︆︌︂︊️",
        "MapperX︍︃︋︉︎︊︈︆︇︋️︇︆️️︌︃︊︊︍︊️︃️︄︍︎︉︃︃︂︉︇︋️︀︎︈︆︊︇︂︋︍︈︍︌︀︀︀︆︉︇︈︉︀︅︄︃︆︉︃︎︂",
        "MapperX︄︄︊️︌︌️︊︉︌︆︃︋︉︁︉︎️️︀︄︄︌︋︊︊︍️︉︎︉︀︎️︄️︆︇︄︅︇︁︅︌︊︊︋︃︉︄︎︆︃︈︆︈️︆︊︇︃︎︋︎",
        "MapperX️︋︁︆︁︎︇︈️︃︂︄︂︋︀︅︆︈︊︅︀︍︆︊︋︈︍︅︌︎︄︎︋︎︆︍︈︈︄︅︆︍︂︈︅︀︇︎︍︉︅︃️️︉︇︂︈︌︅︆︇︅︉",
        "MapperX︁︁︎︄︅︋︌︈︃︋︇︂️︅︂︆︎︈︇︃︊︀︅︎︆︍︊︋︊︅︅︍︅︊︂︂︊︁︃︎︂︎︊︁︉︃︁︄︍︋️︅︆︆︌︈︉️︌︃︅︁︈️",
        "MapperX︇️️︈︊︄︃︈︉︎︇︅︎︀️︄︇︋︌︎︃︉︂︆︂︇︄︉︇️︊︊︂︍︃︎︊︁︍︍︌︁︌️︈︆︀︇︃︈︆︇︆︈︈︎︍️︉︉︅︌︀︃",
        "MapperX︆︊︎︃︉︅︉︎︂︄︆︊︄︅︌︆︉︊︋︎︅︋︈︈︍︍️︈︇︃︌︇︃︃︌︌︃︂︍︈︃︌︁︉︄︎︂︈︌︁︍︇️︅︆︎︋︆︇︇︃︃︆︉",
        "MapperX︆︋︁︃︀︉︁︌︄︅︆︎︀︎︇︁︊︃︇︊︇︃︁︎︉︎︅︈︃︅️︆︎︌︂︋︃︄︃︁️︎︊︊︍︇︉︂︉︉︊︎︅︃︊️︉︈︍︈︆︌︀︁",
        "MapperX︍︈︊︌︁︆︍︂︋︂︉︉︆︊︌︃︁︍︅︉︇︉︃︁︀︄︉️︀︈️︅︂︇️︉︇︂︃︎️︃️︌︂︈︍︇︂︀︋️︇︅︍︃︎︁︀️︋︆︍︋",
        "MapperX︂︇︂︉︀︀︂︎︈︊︎︋︌︂️︉︆︍︃︅︊︎︉️︋︂︈︍︇︎︁︆︋︌︁︅︎︅︃︃︊︅︀︈︁︆︅︆︃︃︉︎︄︃︄︀︇︂︊︂︋︁︆︇",
        "MapperX︁︇︇︋︊︅︂︂︆︄︉︅︃︉︌︇︃︍︅︇︂︁︆︊︇︈︋︎︃︀️︊︄️︉︂︉︃︅︄️︌︋︎︅︈︀︋︀︆︋︈︋︉︀︃︀︀︉︉︍️︀︎",
        "MapperX︍︍️︎️︊︂︌︆︎︇︍︇︀️︊︅️︃️︃︎︌︃︈︃️︀︆︀︄︅️︍︉︍︋︄︋️︍︌︆︆️︀︌︉︎︍︋️︌︌︀︄︌︀︄︁︀︁︇︍",
        "MapperX︆︌︂︊️︌︃︍︌︈︊︊︂︄︊︊︉︇︍︉︂︅︎︉︈︄︃︋︊︌︀︎︀︉︊︅︌︉︈︁︉︉︆︈︁︈︉︀︄︋︆︆︁️︅︍︀︂️︇︂︇︄︆",
        "MapperX︁︃︀︈︆︋︃︊︆︀︀︅︆︈︉︅︎︎︊︎︄︇︍︃︃︃︍︊︅︍︃︌︍︆︃︎︃︁︃︎︋︎️︈︌︃︌︋︃︆︃︊︉︂︌️︈️︊︆︌︀︀︁",
        "MapperX︅︍︅︍︇︇︉︌︈︌︋︄️︂︁︈︅︎︀︃︇︍️︄︃︄︂︇︈︌︈︈︅︃︂︆︂︄︁︆︅︄︁︆︊︅︀︅︊︍︎︉︀︃︀︁︅︇︈︂︋︊︈︌",
        "MapperX︁︄︅︅︈︇︎︂︆︀︅︄︇︁︂︊︊︃️︅︋︀︁︇︇︋︌︈︉︅︁︈︋︊︅︍︈︌︅︆︃︅︄︉︀️︅️︃︅︂︋︌︌︃︌︃︊︍︃︄︃︈︀",
        "MapperX︉︀︆︉︎︇︆︌︀︃︌︈︉︅︊︍︎︋︍︉︈︁︆︀︋︅︉︁︈︀︌︂︄︋︀️︈︄︇︎︄︉︁︀︋︎︀︈︁︊︎︁︂︉︂︆︆️︆︁︃︆︌︅",
        "MapperX️︋️︅︍︉︅︆︅︂︁︌︄︂︊︆︋︋︈︌︈︋︃︍︈️︆︌︀︀︇︀︁︌︅︈︁︀︄︂︂︀︆︊︌︃︊︎︊︆︆️︍︄️︋︈︌︉︄︄︂︋︆",
        "MapperX︎︄︆︋︂︎︈︀︀︎︃︊︆️︊︉︊︈︆︁︇︈︎︅︀︉︊︎︃︊︅︈︇︉︌︊︅︋︆︈︉︋︃︈︌︉︃︄︉︍︉︅︊︋︉︂︅️︎︉︍︂︈︇",
        "MapperX︂︃︇︈︎︎︋︄︁︍︎︆︎︇︂︈︀︉︈︆︀️︀︃︁︀︇︇︍︂︆︁︆︂︂︀︉︌︋︁︎︁︊︁︄︇︃︅︄︍︁︂︍︃️︂︂︌︇︍︁︄︂︀",
        "MapperX︄️︁️︉︉︂︋︎︅︇︄︄︁︄︀︊︆︍︍︃︆︌︎︉︍︃︎︆︁︂︆︇︁︁︌︈︀︂️︉︁︋︉︆︉︃️︍︊︊︋︄︌︆︈︌︉︋︉︂︄︆︊",
        "MapperX︍︅︍︇︈︃︃︆︈︅︋︅︁️︉︁︁︄︈︌︄︄︎︆︌︃︎︁︍︋︅︅︀︍︄︉︍︂︀︈︉︁︃︇︌︀︌️︂︎︂︃︋︃︍︌︇︊︅︂︀️︄︀",
        "MapperX︆︇︇︌︋︍︁︍︇︍︉︂️︎︅︉︎︄︍︄︉︄︀︉︀︈︂︉️︌︄︋︋︅︈︊︂︀︁︊︈︌︃︃︃︋︍︎︊︃︉︅︌︃︆️︈︍︅︈️︇︍︂",
        "MapperX︌️︋︁︈︂︃︀️︌︃︇︈︇︇︅︀︈︍︉︌︉️︋︃️︂︃︅︌︊︎︂︂︅︍︈︍︍︊︇︌︂︋︄️︃️︇︄︈︌︉︄️︁︊️︀︇︄︎︋️",
        "MapperX︂︂︂︊︁︂︄︄︅︎︍︁︅︆︇︊︎︁︁︎︃︃︎︅︃︉︀︎︈︊︈︇︋︂︆︄︆︎︋︆︁︃︉︄︈︀︁︅︈︆︀︈︍︇︄︉︀︆︄︆︃︌︂️",
        "MapperX︍︄︂︋︉︃️︈️︋︆︆︆️︆︈️︎︇︃︀︂︉︌️︀︇︅︈︍︉︃︀︀︉︁︃️︆︈︋︃︁︁︇︉︄︃︁︀︃︁︆︉︉︉︀︌︌︄︇︁︍︁",
        "MapperX︁︎︄︅︈︇︂︈️︅︋︉︆︄️︆︁︉︌︊︆︍︍︍︊️︀︍︅️︎︃︇︂︅︁️︂︀︊︄︁︈️︌︀︊︁️︇︄️︌︂︇︊︍️︈︅︄︌︁︎",
        "MapperX︄︂️︅︁︃︃︌︇︉︉︅︈︄︆︆︇︁︁️︂︀︃︅︁︆︁︌︁︃︋︁️︄︍︎︇️︋︎︇︊︎︄︅︃️︂︇️︀︉︎︊︀︍︃︇︅︌️︄︄︄",
        "MapperX︎️︋︂︂︂︌︈︉︍︀︋︃︉︆︋︈︅︊︈️︉︋︇︍︃︌︃︄︇︋︋︋︎︃︊︌️︂︂︍︃︂︊︈︉︎︈︌︋︎︉︊︋︎︍︄︁︆︈︍︈︋︄",
        "MapperX︅︂︋︍︉︀︊︅︇︄︂︃︉︇︉︆︀︁︈️︄︍︆︂︁︃︅︄︀︍︄︋︌︌︆︌︂︍︀︅︆︎︉︃︌︈︄︊︋︅︆️︅︊︋︉︋︋︆︋︂︂︁︍",
        "MapperX︃︉︃︋︎︆︀︊︉︂︄︂︋︅︄︅︁︊︋︂︉︁︁︍︊︇︃︀︇︊︇︅︎︌︈︅︋︁︎︄︆️︇︂︃︈︍︂︉︊︈️︎︄︆︌︇︈︀︄︄︆︁︀",
        "MapperX︊︇︆︌️︇︌︀︎︈︃︈︉︂︇︊︆︋︀︀︄︁︎︁︂️︎︁︈︉︅️︉︆︇️︄︄︄︁︀︋︅︌︈︈︍️️︊︂︍︋︁️︄︌︌︍︉︈︋︅︆",
        "MapperX︎︂️︉︄︅︌︄︉︌︊︊︆︎︅︈︊︆︀︌︍︋︀︇︋︊️︀︅︃︂︉︁︇︋︋︂︋︎︎︊︊︉︉︍︋︀︆️︊︃︌️︃︁︊︄︊︉︌︎︋︉︊",
        "MapperX︁︀︈️️︅︊︈︄︋︀️︅︇︈︍︌︅︄︃︆︇︋︂︄︅︍︌︊︇︇︅︎︈︆︌︎︂️️︋︇︎︋︊︎︁︄︀︋︇︊︇︂︌︌︉︎︃︎︉︊︊︈",
        "MapperX︎︀︂︈︎︌︉︊︊︅︋︇︂︍︌︉︋︊︊︁︎︁︃︅︉︈︃︎︂️︀︊︃︃︂️︃︈︌︄︊︈︀︃︆︈︇︌︆︇︉︋︈︃︈︈︂︇︊︈︋︇️︃",
        "MapperX︈︎︄︈︆︅︎︌︌︉︃️︇︌︈︋︄︈︎︎︈︅︅︌︇︌︇︃︃︍︉︃︍︌︉︉︂︃︄︆︇︍︋︎︈︈︌︃︆︊︅︃︎︃︂︀︋︉︌︆︀︍︀︋",
        "MapperX︃︆︃︆︍︄︎︉︆︂︉︊︊︀︈︆︈️︋︃︊︈︄︅︂︊︁︂︎︎︌︁︉︉︃︂︊︋️︃︉︃︄︎︃︄️︍︅︅︁︆︇︆︊︂︄︆︆︆︊︅︌︆",
        "MapperX︌︍︁︇️︅︃︌︇︋︄︊︄︋︈︎︆︈︎︍︃︍︈︄︎️︉︁︅︄︊︉︎︄︃︁︄︃︆︋︇︆︇︀︋︈︎︇︄︀︅︌︃︆︆︀︍︈︅︀︂︆︃︍",
        "MapperX︈️︍︍︉︍︅︉︉︇︆︃︌︊︇︌︇︅︋︅︉︇︉︂︇︀︌︋️︀︆︎︆︈︄︌︃︄︈︆️︀︋︌︂︂︉︄︁︇︄︄︊︇︌︊︈︁︀︀︊︅︉︈",
        "MapperX︆︎︈︂︌︄︅︊︅︀︊︌︄︌︇︂︅︃︁︁︋️︅️︌︎︇︉︌︆︄︀︃︋︆︉︀︀︂︂︋︉︍︈︁️︅︇︁︋︌︂︋︊︎︃︄︎︉︎︁︀︍︄",
        "MapperX︄︁︂︍︃︂︌︋️️︂︃️︁︎︅︉︁︋︋︍︌︇︅︂︃︃︆︀︉︅︂︀︃︅︅︆︊︌︍︋︁︄︂︊︌︂︎️︈︅︅︉︇︉︄︎️︎︅︉️︋︉",
        "MapperX︊️︃︋︆︄︁︆︁️️︉︀︋︀︈︎︁︃︉︊︄︄︎︀︁︁︀︁︀︆︋︍︄︆︀︉︁︁︃︉︀︌︉︉︎︎︅︄︄︃︃︅︋︆︋︂︆︎︃︀︌︋︊",
        "MapperX︇︎︁︉︎︊︅︄︋︇︆︋︇︍︄︊︉︎️︀︁︀︀︂︅︂︄︊︈️︍️︆︄︇︀︀︆︅︆︉︆︉︇︃︇︌︂︎︍️️︊︁︂︈︆︆︊︂︋︌︆︄",
        "MapperX︉︌︆︂︁︀︊︎︂︅︍︍︇️︀︌︄︅︍︃︍︋︎︈︅︋︈︋︊︄︅︄︈️︃︉︆︋︃︆︊︊︈︌︋︊︂︀︎︅︆︂︅︌︇︃︅︌︉︄︇︊︉︇",
        "MapperX︉︂︎︇︇︍︆︆︅︈︎︍︌︂︍︂︊︃︉︀︉︌️︀️︁︊︉︍︉︌︈︂︈︎︈︌︀︂︋︄︋︉︉︎︇︍︎︄︅︂︇️︁︈︂︋︋︇︈︁︋︋︎",
        "MapperX︈︉︊︊︂︆︋︍︇︎︂︇︄︆︃︊︈︅️︉️︅︇︌️️︆︁︇︍︎︀︇︈︇︎︉︊︍️︆︁︈︊︊︋︀︀︌︁︇︁︊︆︃︂︄︄︁︈︍︌︈︆",
        "MapperX︀︆︁︇︉︂︃︈︃︉︎︁︎︊︍︋︎︈︆️︁︋️︃︃︈︍︁︃︍︊︎︈︉︀︄︃︍︉︈︍︍︋︉︅︌︀︀︍︍︊︊︅︊︌︈︆︎︀︉︍︄︅︀",
        "MapperX︁︊️︄︁︍︅︋︊︍︉︉︊︇︄︊︅︈︍️︂︅︌️︇︎︇︉︂︅︁︌︄︋︀︄︅︊︍︃︄︆︁︎︀︊︂︇︇︊︉︀︄︍︉︄︀︋︃︊︂︋︌︃",
        "MapperX︂︋︌︌️︇︆︎︌︁︌︂︉︎︋︄︎︎︃︃︊︈︀︂︈︁︍︆︃︊︈︊︈︇︇︃︃︃︉︃︋︌︅︁︅︂︀︋︉︊︉︉︈︃︍︎︆︃︇︉︌︃︆︈",
        "MapperX️︇︁︋️︋️︎︉︉︂︆︁︋︊︆︃︊︎︈︊︁︌︆️︃︇︎︁︈︉︈︈︍︊️︀︈︉︇︆︊︄︀︄︂︄︄︃︇︍︆︁︋︅︆︈︄︁︀︀︋︉︄",
        "MapperX︇︅︄︍︍︁︅︊︁︃︍︌︉︉︄︍︃︃︆︎︍︅︁︆︊︆︀︇︇︀︄︌︍︇️︌︃︍️︂️︇︎︍︊️︁︋︋︎︂️︀︂︅️︇︆︌︎︊︅️︎",
        "MapperX︆︀︃︎️︉︍︆️︍️︎︎️︋︉︉︄︂︋︌︈︈︇︂︈︆︉︃︆︋︎︍︀︈︌︉︇︄︅︁︆︎︎︆︈︇︉︄︈︂︄︍︍︂︎︅︆︈︅︁︋︍︆",
        "MapperX︅︋︋︁︎︀︃︋︅︋︉︍︌︅︆︍︉︈︇︊︉︃︋︊︎︀︇︂︇︄︌︆︇︅︃︆︆︍︊︀︂︉︍︀︁︎︅︃︋︊︃︁︁︄︆︂︅︈︁︀︎︍︊︈",
        "MapperX︌︄︇︎︁︎︊︈︍︎️︂️︇︋︃︌︉️︉︄︆︈︊︆︎︉︁︉︊︉︃︃︁︊︈︀︃︋︂︊︈︀️︌︌︊︋︍︀︋︇︃︆︆︅︁︉︁︆︍︄︇︉",
        "MapperX︌︉︈︊︆︀︆️︎︉︌︌︍︊︆︋︁︌︂︆︊︂︅︄︁︇︁︉︀︊️︄︅︉︇︃︋︍︀︆︎︀︊︃︀︈︀︁︁︉︁︉︈︄︍︇︇︃︉︈︆︀︁︊",
        "MapperX︂︈︆️︅︅︌︌︍︉︆︄︌︁︋︎︈︆︉︂︂︍︀️︋︃︊︎︃︅︂︉︇︎︆︃︍︋︊︌︀︊︂︇︎︇︍︈︅︃︅︊︋︈︎︁︀︉︉︎︆︀︃︋",
        "MapperX︀︈︅︉︎︀︋️︉︋︌︈︊︉︆︎︃︌︁︄︅︉︉︌︈︊︄︈︃︂️︊︇︁︀︋︍︎︂︌︎︉︆︅︂︈︃︆️️︁︂︍︈︀︇︉︂︂︂︇︉︄︌",
        "MapperX︋︎︄︀︌︈︈︈︀︂︉︋︍︄︉︎︂︂︎︉︀️︃︃︊︂︁︇︅︅︈︉︂︂︈︉︋︅︇︈︂︉︍︌︌︉︋︌︅︈︇︋︁︅︄︆︀︇︍︅︉︁︁︂",
        "MapperX︍︅︇︀︌︎︅︁︇︇︅︅︂︌︌︊︁︍︁︍︍︆︉︄︀️︄︍︂︅︈︆︌︃︉︆︇︌︍︁︀︎︉︇︆︀︄︃︊︌︅︈︊︆︍︉︁︌︁︈︅︊︁︍",
        "MapperX️️️︊︆︈︄︄︌︁︎︈︈︃︃︄︋︁︁︁︀︅︁︂︎︄︊︍︋️︊︃︌︋︉︀︄︅︆︊️️︋︌︅︈︄︊︉︁︎︌︋︆︌︍︎️︂︊︈︍︎︈",
        "MapperX︅︎︊︃︌︌︃︊︍︊︃︍︎︁︍︍︇︇︅︀️︌︅︅︃︅︃︇︄︃︉︌︁︆︈︃︁︋︉️︋︉︃︅︈︆︇︇︌︌︊︆︆︈︉︁︎︍︈︎︆︉︅️",
        "MapperX︁️︅︊︆︅︅︇️︆︅︆︇︀︌︊︈︅︃︈︅︅︆︃︌︆︆︁︎️︆︁︉︉︉︎️︌︊︉︃︃︆︀️︁︂︄︉︃︉︎︎︉️︋︉︆︂︍︎︈︋︋",
        "MapperX︈︁︋︀︃︍︀︊︅︂︄︁︉︄︆︂︀︄︂︆︁️︌︁︋︆︋︁︅︇︍︅️︎︅︍︇︊︎︇︊︎︃️︈︀︈︊︃︊︋︎︉︎︋︆️︆︅︍️︅︉︉",
        "MapperX︀︎︄︌︁︌︀︋︍︁︆︊︊︄︂︆︁️︌︀︂︃︈️︄︂️︍︀︇︋︆︂︎︀︅︆︈️︀︅︊︊︄︌︂︄︍︁️︅︈︍︅︄︊︁︀️︋︉︋︂︆",
        "MapperX︌︎︅︎︁︎︎︎︆︃︅︈︎︊︈️︃️︌️️︃︅︀︈︃︎︆︄︄︆️️︈︊︌︆︉︆︆︋︄︃︅︉︍️︍︉︂︍︋︃︌︊︃︋︎︌︈︍︍︁︆",
        "MapperX︅︇︎︃︍︅︇︋︀︆︂︃︌️︂︋︀︅︌︁︀︂︎︌︉︇︊︆︂︊︅︃️︍︁️️︁︇︂︌︈︁︁︋︁︎︄︍︁︈︆︀︆︆︂︆︊︉︅︊︋︀︉",
        "MapperX︆︋︈︊️︌︉︌︇︂︋︁︁︆︎︀︋︍︁︋︀︉︎︇︆︄︆︃︍︊︌︂︇︎︊︃︎︈️︉︁︋︅︅︁︋︇︀︍︆︃︂︎︍︃︁︃︋︌︀︅︄︊︆",
        "MapperX︆︀︉︇︄︎︅︃︈︍︈︋️︊︉︈︀︀︄︇︆︄︋︀︁︍︎︋︅︊︂︃︉︇︆︎︊︂︃︎︅︃️︉︄︇︍︆︁️︉︄︄︌︊︃︉︆︄︂︍︀︄︈",
        "MapperX︍︊︆︁︂︍︀︌︋︃︆︂️︄︉️︅︉︂︁︀︇︁︋︌︉︍︁︈︃︂︁︈︌︂︇︌︃︈︅︆︋︄︋︊︆︇︃︊︀︎︂︋︎︄︃︇︆︈︎︁︉︁︉"
    };

    /** This utility has a single static entry point. */
    private MapperX() {}

    /** Immutable launch request; no robot commands are sent by the launcher. */
    private static final class LaunchPlan {
        /** Project selected explicitly or discovered from the working directory. */
        final Path project;
        /** Native application command with arguments kept as separate values. */
        final List<String> command;

        /** Save the fully resolved project and command. */
        LaunchPlan(Path project, List<String> command) {
            this.project = project;
            this.command = command;
        }

        /** Start the desktop app with its project environment already configured. */
        Process start() throws IOException {
            ProcessBuilder process = new ProcessBuilder(command);
            process.directory(project.toFile());
            process.environment().put("MAPPERX_PROJECT", project.toString());
            process.inheritIO();
            return process.start();
        }
    }

    /** Return an option value and reject incomplete command lines. */
    private static String option(List<String> args, String name) {
        int index = args.indexOf(name);
        if (index < 0) return null;
        if (index + 1 >= args.size() || args.get(index + 1).startsWith("--")) {
            throw new IllegalArgumentException("Missing value for " + name);
        }
        return args.get(index + 1);
    }

    /** Locate the project in the requested folder or one of its parents. */
    private static Path findProject(String selected) throws IOException {
        Path start = Paths.get(selected == null ? "." : selected).toAbsolutePath().normalize();
        for (Path folder = start; folder != null; folder = folder.getParent()) {
            boolean matches = true;
            for (String required : PROJECT_FILES) {
                if (!Files.exists(folder.resolve(required))) matches = false;
            }
            if (matches) return folder.toRealPath();
            if (selected != null) break;
        }
        throw new IOException("Select the robot project folder containing field_planner.py, "
                + "build.gradle and src/main/java: --project <folder>");
    }

    /** Resolve the actual executable inside a macOS bundle or a Windows build. */
    private static Path executable(Path candidate, boolean mac) {
        if (mac && candidate.toString().endsWith(".app")) {
            return candidate.resolve("Contents/MacOS/Shuffleboard");
        }
        return candidate;
    }

    /** Build a native launch plan using an explicit app or the project output. */
    private static LaunchPlan plan(Path project, String selectedApp) throws IOException {
        String os = System.getProperty("os.name").toLowerCase(Locale.ROOT);
        boolean mac = os.contains("mac");
        boolean windows = os.contains("win");
        if (!mac && !windows) {
            throw new IOException("The desktop build requires Windows or macOS.");
        }
        List<Path> candidates = new ArrayList<>();
        if (selectedApp != null) {
            candidates.add(executable(Paths.get(selectedApp).toAbsolutePath().normalize(), mac));
        } else {
            for (String relative : DESKTOP_DIRS) {
                Path folder = project.resolve(relative);
                candidates.add(executable(folder.resolve(mac ? "Shuffleboard.app" : "Shuffleboard.exe"), mac));
            }
            String configured = System.getenv("MAPPERX_APP");
            if (configured != null && !configured.trim().isEmpty()) {
                candidates.add(executable(Paths.get(configured).toAbsolutePath().normalize(), mac));
            }
            if (mac) {
                candidates.add(executable(Paths.get("/Applications/Shuffleboard.app"), true));
                candidates.add(executable(Paths.get(System.getProperty("user.home"),
                        "Applications/Shuffleboard.app"), true));
            }
        }
        for (Path candidate : candidates) {
            if (Files.isRegularFile(candidate) && (windows || Files.isExecutable(candidate))) {
                return new LaunchPlan(project, Arrays.asList(candidate.toRealPath().toString()));
            }
        }
        throw new IOException("Shuffleboard was not found. Build or copy the desktop app, "
                + "or specify --app <Shuffleboard.exe or Shuffleboard.app>. Checked: " + candidates);
    }

    /** Describe the selected project and executable without opening the interface. */
    private static void describe(LaunchPlan plan) {
        System.out.println("MapperX project: " + plan.project);
        System.out.println("Desktop command: " + plan.command);
        System.out.println("Source identity: " + Integer.toUnsignedString(Arrays.hashCode(BUILD_IDS), 16));
    }

    /** Parse the command line, resolve the project and start the desktop interface. */
    public static void main(String[] values) {
        List<String> args = Arrays.asList(values);
        if (args.contains("--help")) {
            System.out.println("java MapperX.java [--project folder] [--app application] [--check]");
            System.out.println("--check validates the launch plan without starting the application.");
            return;
        }
        try {
            for (int i = 0; i < args.size(); i++) {
                String arg = args.get(i);
                if (arg.equals("--project") || arg.equals("--app")) {
                    option(args, arg);
                    i++;
                } else if (!arg.equals("--check")) {
                    throw new IllegalArgumentException("Unknown option: " + arg);
                }
            }
            Path project = findProject(option(args, "--project"));
            LaunchPlan launch = plan(project, option(args, "--app"));
            describe(launch);
            if (!args.contains("--check")) launch.start();
        } catch (IOException | IllegalArgumentException error) {
            System.err.println("MapperX: " + error.getMessage());
            System.exit(1);
        }
    }
}

