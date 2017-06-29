# -*- coding: utf-8 -*- 
from sqlalchemy import create_engine, Column, Integer, String, Sequence, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm.session import sessionmaker
import rospy, os

## Declarative database synonym
Base = declarative_base()

class DroneMove(Base):
    __tablename__ = 'drone_move'
    id    = Column(Integer, Sequence('drone_move_id_seq'), primary_key=True)
    drone = Column(String)
    point = Column(String)

class DroneFree(Base):
    __tablename__ = 'drone_free'
    id    = Column(Integer, Sequence('drone_free_id_seq'), primary_key=True)
    drone = Column(String, unique=True)

    def __init__(self, adapter):
        self.drone = adapter

def spawn_db():
    db = create_engine(os.environ['DB_CONN_STRING'], client_encoding='utf8')
    return sessionmaker(bind=db)()

def messenger_drone_free(adapter):
    db = spawn_db()
    if not db.query(DroneFree).filter_by(drone=adapter).first():
        db.add(DroneFree(adapter))
        db.commit()

def messenger_mission_gen(adapter):
    db = spawn_db()

    while not rospy.is_shutdown():
        try:
            if len(db.query(DroneMove).filter_by(drone=adapter).all()) > 0:
                act = db.query(DroneMove).filter_by(drone=adapter).first()
                if act.point == 'A':
                    yield 2
                elif act.point == 'B':
                    yield 3
                elif act.point == 'C':
                    yield 4
                elif act.point == 'Home':
                    yield (-1)
                db.query(DroneMove).filter_by(drone=adapter).delete()
                db.commit()
        except:
            rospy.logerr('Unable to get movement from FBMessenger')
        rospy.sleep(1)
