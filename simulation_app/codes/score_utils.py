import traceback
import numpy as np
from omni.isaac.core.utils import viewports, stage
from omni.kit.commands import execute
import carb
from pxr import Sdf, Gf
import omni.graph.core as og
import omni.usd
import usdrt

#   setup_score_scope()를 대신해서 개발.
#   update된 현황판에 필요한 정보로 재구성함 
def _create_scoreboard_scope_and_attribute():
    #! 점수 확인용 Scope 생성
    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Scoreboard/score",
        attributes={},
        select_new_prim=True,
    )

    #! 점수 확인용 Attribute 생성
    execute(
        "CreateUsdAttributeOnPath",
        attr_path=Sdf.Path("/Scoreboard/score.stage1"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.Double,
    )

    execute(
        "CreateUsdAttributeOnPath",
        attr_path=Sdf.Path("/Scoreboard/score.stage2"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.Double,
    )

    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Scoreboard/recycling",
        attributes={},
        select_new_prim=True,
    )
    
    execute(
        "CreateUsdAttributeOnPath",
        attr_path=Sdf.Path("/Scoreboard/recycling.recycled"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.Int,
    )

    execute(
        "CreateUsdAttributeOnPath",
        attr_path=Sdf.Path("/Scoreboard/recycling.total"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.Int,
    )


    execute(
        "CreatePrimWithDefaultXform",
        prim_type="Scope",
        prim_path="/Scoreboard/team",
        attributes={},
        select_new_prim=True,
    )

    execute(
        "CreateUsdAttributeOnPath",
        attr_path=Sdf.Path("/Scoreboard/team.name"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.String,
    )

    execute(    
        "CreateUsdAttributeOnPath", 
        attr_path=Sdf.Path("/Scoreboard/team.stage"),
        custom=True,
        variability=Sdf.VariabilityVarying,
        usd_context_name="",
        attr_type=Sdf.ValueTypeNames.Int,
    )


def setup_scoreboard(simulation_app):
    _create_scoreboard_scope_and_attribute()
    simulation_app.update()

    _setup_scoreboard_omnigraph()
    simulation_app.update()



def _set_scoreboard_timeout_color(color): 
    # Prim 가져오기
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(f"{GRAPH_PATH}/{NODE_TIME_ELLEPSED}")

    # 속성 읽기
    if prim is not None and prim.HasAttribute('inputs:textColor'):
        attr = prim.GetAttribute('inputs:textColor')
        attr.Set(color)


def _set_scoreboard_value(type_name, attribute_name, value): 
    # Prim 가져오기
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(f"/Scoreboard/{type_name}")

    # 속성 읽기
    if prim is not None and prim.HasAttribute(attribute_name):
        attr = prim.GetAttribute(attribute_name)
        attr.Set(value)
        
def _read_scoreboard_value(type_name, attribute_name): 
    # Prim 가져오기
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(f"/Scoreboard/{type_name}")

    # 속성 읽기
    if prim is not None and prim.HasAttribute(attribute_name):
        attr = prim.GetAttribute(attribute_name)
        value = attr.Get()
        return value
    
    return None
        
def set_scoreboard_recycling(recycled, total=None):        
    if total is not None:
        _set_scoreboard_value("recycling", "total", total)
    if recycled is None:
        _set_scoreboard_value("recycling", "recycled", 0)
    else:
        _set_scoreboard_value("recycling", "recycled", recycled)

def read_scoreboard_recycling_total():
    return _read_scoreboard_value("recycling", "total") 

def read_scoreboard_recycling_recycled():
    return _read_scoreboard_value("recycling", "recycled") 

def set_scoreboard_score_stage1(value):
    _set_scoreboard_value("score", "stage1", value)

def set_scoreboard_score_stage2(value):  
    _set_scoreboard_value("score", "stage2", value)

def read_scoreboard_score_stage1():
    return _read_scoreboard_value("score", "stage1")

def read_scoreboard_score_stage2():
    return _read_scoreboard_value("score", "stage2")

def set_scoreboard_team_info(teamName, teamStage):
    _set_scoreboard_value("team", "name", teamName)
    _set_scoreboard_value("team", "stage", teamStage)

def read_scoreboard_team_info():
    team_name = _read_scoreboard_value("team", "name")
    team_stage = _read_scoreboard_value("team", "stage")

    return team_name, team_stage



GRAPH_PATH = "/CompetitorScoreBoard"

NODE_BACKGROUND_TEAM = "ScoreboardBackgroundTeam"
NODE_BACKGROUND_SCORE = "ScoreboardBackgroundScore"

NODE_TEAM_NAME = "TeamName"
NODE_TEAM_STAGE = "TeamStage"
NODE_LEVEL_SCORE = "LevelScore"
NODE_RECYCLE_STATUS = "RecycleStatus"

NODE_TIME_ELLEPSED = "TimeEllepsed"

NODE_TYPE = "omni.graph.visualization.nodes.DrawScreenSpaceText"

SCOREBOARD_BOX_WIDTH = 300
SCOREBOARD_BACKGROUND_PADDING = 10.0

SCOREBOARD_BOX_X = SCOREBOARD_BOX_WIDTH + SCOREBOARD_BACKGROUND_PADDING * 3

SCOREBOARD_FONT_SIZE_TITLE = 22
SCOREBOARD_FONT_SIZE_CONTENT = 18

SCOREBOARD_BOX_TEAM_BACKGROUND_Y = 40.0
SCOREBOARD_BOX_TEAM_BACKGROUND_HEIGHT = SCOREBOARD_BACKGROUND_PADDING * 2 + SCOREBOARD_FONT_SIZE_TITLE

SCOREBOARD_BOX_SCORE_BACKGROUND_Y = SCOREBOARD_BOX_TEAM_BACKGROUND_Y + SCOREBOARD_BOX_TEAM_BACKGROUND_HEIGHT + SCOREBOARD_BACKGROUND_PADDING

SCOREBOARD_BOX_SCORE_WIDTH = SCOREBOARD_BOX_WIDTH - SCOREBOARD_BACKGROUND_PADDING * 2
SCOREBOARD_BOX_SCORE_GAP = 10
SCOREBOARD_BOX_SCORE_OUTER_HEIGHT = SCOREBOARD_FONT_SIZE_CONTENT + SCOREBOARD_BOX_SCORE_GAP

SCOREBOARD_BOX_SCORE_BASE_Y = SCOREBOARD_BOX_SCORE_BACKGROUND_Y + SCOREBOARD_BACKGROUND_PADDING
SCOREBOARD_BOX_SCORE_BACKGROUND_HEIGHT = SCOREBOARD_BOX_SCORE_OUTER_HEIGHT * 4 + SCOREBOARD_BACKGROUND_PADDING


COLOR_BACKGROUND = Gf.Vec4d(0.2, 0.2, 0.2, 0.7)
COLOR_TRANSPARENT = Gf.Vec4d(0.0, 0.0, 0.0, 0.0)
COLOR_TEXT_HURRYUP = Gf.Vec4d(1.0, 0.2, 0.2, 0.7)


def _get_percent_position(view_width, view_height, x, y):
    percent_x = (view_width - x) / view_width
    percent_y = y / view_height
    return Gf.Vec2d(percent_x * 100.0, percent_y * 100.0)
    

def _setup_scoreboard_omnigraph():
    #! 점수 확인용 Omnigraph Scoreboard 생성

    try: 
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    (
                        "on_playback_tick", 
                        "omni.graph.action.OnPlaybackTick"
                    ),
                    (
                        NODE_BACKGROUND_TEAM,
                        NODE_TYPE,
                    ),                    
                    (
                        NODE_BACKGROUND_SCORE,
                        NODE_TYPE,
                    ),                    
                    (
                        NODE_TEAM_NAME,
                        NODE_TYPE,
                    ),
                    (
                        NODE_TEAM_STAGE,
                        NODE_TYPE,
                    ),
                    (
                        NODE_LEVEL_SCORE,
                        NODE_TYPE,
                    ),
                    (
                        NODE_RECYCLE_STATUS,
                        NODE_TYPE,
                    ),
                    (
                        NODE_TIME_ELLEPSED, 
                        NODE_TYPE,
                    )   
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_BACKGROUND_TEAM}.inputs:execIn",
                    ),                    
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_BACKGROUND_SCORE}.inputs:execIn",
                    ),                    
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_TEAM_NAME}.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_TEAM_STAGE}.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_LEVEL_SCORE}.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_RECYCLE_STATUS}.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        f"{NODE_TIME_ELLEPSED}.inputs:execIn",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (f"{NODE_BACKGROUND_TEAM}.inputs:text", " "),
                    (f"{NODE_BACKGROUND_TEAM}.inputs:backgroundColor", COLOR_BACKGROUND),
                    (f"{NODE_BACKGROUND_TEAM}.inputs:size", SCOREBOARD_BOX_TEAM_BACKGROUND_HEIGHT),
                    (f"{NODE_BACKGROUND_TEAM}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_BACKGROUND_TEAM}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X, SCOREBOARD_BOX_TEAM_BACKGROUND_Y)),

                    (f"{NODE_BACKGROUND_SCORE}.inputs:text", " "),
                    (f"{NODE_BACKGROUND_SCORE}.inputs:backgroundColor", COLOR_BACKGROUND),
                    (f"{NODE_BACKGROUND_SCORE}.inputs:size", SCOREBOARD_BOX_SCORE_BACKGROUND_HEIGHT),
                    (f"{NODE_BACKGROUND_SCORE}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_BACKGROUND_SCORE}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X, SCOREBOARD_BOX_SCORE_BACKGROUND_Y)),

                    (f"{NODE_TEAM_NAME}.inputs:text", "team name"),
                    (f"{NODE_TEAM_NAME}.inputs:backgroundColor", COLOR_TRANSPARENT),
                    (f"{NODE_TEAM_NAME}.inputs:size", SCOREBOARD_FONT_SIZE_TITLE),
                    (f"{NODE_TEAM_NAME}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_TEAM_NAME}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X+SCOREBOARD_BACKGROUND_PADDING, SCOREBOARD_BOX_SCORE_BASE_Y)),

                    (f"{NODE_TEAM_STAGE}.inputs:text", "team stage"),
                    (f"{NODE_TEAM_STAGE}.inputs:backgroundColor", COLOR_TRANSPARENT),
                    (f"{NODE_TEAM_STAGE}.inputs:size", SCOREBOARD_FONT_SIZE_CONTENT),
                    (f"{NODE_TEAM_STAGE}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_TEAM_STAGE}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X+SCOREBOARD_BACKGROUND_PADDING, SCOREBOARD_BOX_SCORE_BASE_Y + SCOREBOARD_BOX_SCORE_OUTER_HEIGHT)),

                    (f"{NODE_LEVEL_SCORE}.inputs:text", "score"),
                    (f"{NODE_LEVEL_SCORE}.inputs:backgroundColor", COLOR_TRANSPARENT),
                    (f"{NODE_LEVEL_SCORE}.inputs:size", SCOREBOARD_FONT_SIZE_CONTENT),
                    (f"{NODE_LEVEL_SCORE}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_LEVEL_SCORE}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X+SCOREBOARD_BACKGROUND_PADDING, SCOREBOARD_BOX_SCORE_BASE_Y + SCOREBOARD_BOX_SCORE_OUTER_HEIGHT*2)),

                    (f"{NODE_RECYCLE_STATUS}.inputs:text", "recycling"),
                    (f"{NODE_RECYCLE_STATUS}.inputs:backgroundColor", COLOR_TRANSPARENT),
                    (f"{NODE_RECYCLE_STATUS}.inputs:size", SCOREBOARD_FONT_SIZE_CONTENT),
                    (f"{NODE_RECYCLE_STATUS}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_RECYCLE_STATUS}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X+SCOREBOARD_BACKGROUND_PADDING, SCOREBOARD_BOX_SCORE_BASE_Y + SCOREBOARD_BOX_SCORE_OUTER_HEIGHT*3)),

                    (f"{NODE_TIME_ELLEPSED}.inputs:text", "time elapsed"),
                    (f"{NODE_TIME_ELLEPSED}.inputs:backgroundColor", COLOR_TRANSPARENT),
                    (f"{NODE_TIME_ELLEPSED}.inputs:size", SCOREBOARD_FONT_SIZE_CONTENT),
                    (f"{NODE_TIME_ELLEPSED}.inputs:boxWidth", SCOREBOARD_BOX_WIDTH),
                    (f"{NODE_TIME_ELLEPSED}.inputs:position", Gf.Vec2d(SCOREBOARD_BOX_X+SCOREBOARD_BACKGROUND_PADDING, SCOREBOARD_BOX_SCORE_BASE_Y + SCOREBOARD_BOX_SCORE_OUTER_HEIGHT*4)),
                ]
            }
        )


    except Exception as e:
        traceback.print_exc()
  

import omni.ui as ui


def update_scoreboard(time_elapsed, time_constraint):
    #! 점수 확인용 Omnigraph Scoreboard 업데이트

    stage1_score = read_scoreboard_score_stage1()
    stage2_score = read_scoreboard_score_stage2()
    recycled = read_scoreboard_recycling_recycled()
    total_objects = read_scoreboard_recycling_total()
    team_name, team_stage = read_scoreboard_team_info()


    if team_name is None:
        team_name = "Waiting for participant ..."
        recycling_str = ""
        applied_to = ""
        score_str = ""
        time_elapsed_str = ""

        # return 
    else:
        team_name_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_TEAM_NAME}.inputs:text")
        if team_name_attr is not None:
            if len(team_name) > 30:
                team_name = team_name[:27] + "..."
            team_name_attr.set(team_name)

        applied_to = "Participation: "
        score_str = "            Score: "
        if team_stage == 1:
            applied_to = "Participation: Stage 1"

            if stage1_score is not None:
                score_str = f"            Score: S1 = {stage1_score:.2f}"
            else:
                score_str = f"            Score: S1 = -"

        elif team_stage == 2:
            applied_to = "Participation: Stage 1 & Stage 2"

            string_stage1_score = "-" if stage1_score is None else f"{stage1_score:.2f}"
            if stage2_score is None:
                score_str = f"            Score: S1 = {string_stage1_score}, S2 = -"
            else:
                score_str = f"            Score: S1 = {string_stage1_score}, S2 = {stage2_score:.2f}"


        recycling_str = f""
        if team_stage == 2:
            if total_objects is None:
                recycling_str = f"      Recycling: - / - items collected"
            elif recycled is None:
                recycling_str = f"      Recycling: 0 / {total_objects} items collected"
            else:
                recycling_str = f"      Recycling: {recycled} / {total_objects} items collected"

        if time_elapsed is None :
            time_elapsed = 0
        else:
            time_elapsed = time_elapsed.total_seconds()
        
        time_remaining = time_constraint - time_elapsed
        time_remaining = max(time_remaining, 0)

        minutes = int(time_remaining // 60)
        seconds = int(time_remaining % 60)
        milliseconds = int((time_remaining - int(time_remaining)) * 1000)
        time_elapsed_str = f"      Time Left: {minutes:02d}:{seconds:02d}:{milliseconds:03d}"


        if time_remaining < 10:
            _set_scoreboard_timeout_color(COLOR_TEXT_HURRYUP)

    window = ui.Workspace.get_window("Viewport")
    
    viewport_width = 1024
    viewport_height = 768
    if window is not None:
        viewport_width = window.width
        viewport_height = window.height

    try: 
        team_name_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_TEAM_NAME}.inputs:text")
        if team_name_attr is not None:
            team_name_attr.set(f"{team_name}")

        aplied_stage_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_TEAM_STAGE}.inputs:text")
        if aplied_stage_attr is not None:
            aplied_stage_attr.set(applied_to)

        score_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_LEVEL_SCORE}.inputs:text")
        if score_attr is not None:
            score_attr.set(score_str)

        recycled_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_RECYCLE_STATUS}.inputs:text")
        if recycled_attr is not None:
            recycled_attr.set(recycling_str)

        time_elapsed_attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_TIME_ELLEPSED}.inputs:text")
        if time_elapsed_attr is not None:
            time_elapsed_attr.set(time_elapsed_str)

        background_position = _get_percent_position(viewport_width, viewport_height, SCOREBOARD_BOX_X, SCOREBOARD_BOX_TEAM_BACKGROUND_Y)
        attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_BACKGROUND_TEAM}.inputs:position")
        if attr is not None:
            attr.set(background_position)

        background_position = _get_percent_position(viewport_width, viewport_height, SCOREBOARD_BOX_X, SCOREBOARD_BOX_SCORE_BACKGROUND_Y)
        attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_BACKGROUND_SCORE}.inputs:position")
        if attr is not None:
            attr.set(background_position)


        position_x = SCOREBOARD_BOX_X - SCOREBOARD_BACKGROUND_PADDING
        position_y = SCOREBOARD_BOX_TEAM_BACKGROUND_Y + SCOREBOARD_BACKGROUND_PADDING

        ## Team Name
        team_name_position = _get_percent_position(viewport_width, viewport_height, position_x, position_y)
        attr = og.Controller.attribute(f"{GRAPH_PATH}/{NODE_TEAM_NAME}.inputs:position")
        if attr is not None:
            attr.set(team_name_position)

        ## Scoreboard contents: Team appied stage, score, recycling, time left
        position_y = SCOREBOARD_BOX_SCORE_BASE_Y
        for node in [NODE_TEAM_STAGE, NODE_LEVEL_SCORE, NODE_RECYCLE_STATUS, NODE_TIME_ELLEPSED]:
            box_position = _get_percent_position(viewport_width, viewport_height, position_x, position_y)

            attr = og.Controller.attribute(f"{GRAPH_PATH}/{node}.inputs:position")
            if attr is not None:
                attr.set(box_position)

            position_y += SCOREBOARD_BOX_SCORE_OUTER_HEIGHT


    except Exception as e:
        traceback.print_exc()
