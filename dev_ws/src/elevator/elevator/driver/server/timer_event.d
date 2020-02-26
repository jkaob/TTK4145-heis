import  std.stdio,
        std.conv,
        std.variant,
        std.concurrency,
        std.algorithm,
        std.range,
        std.typecons,
        std.typetuple,
        std.datetime,
        std.traits,
        core.time,
        core.thread,
        core.sync.mutex;
    
/+    
struct DeleteEvent(T){
    alias T type;
}

void testThread(){

    writeln("testThread thisTid: ", thisTid);

    while(true){
        receive(
            (int i){
                writeln("    testThread received int: ", i);
                if(i > 5){
                    i = 1;
                    addEvent(ownerTid, 50.msecs, "should not arrive");
                    ownerTid.send(DeleteEvent!string());
                }
                addEvent(thisTid, i.seconds, i+1);
            },
            (string s){
                writeln("    testThread received string: ", s);
            },
            (Variant v){
                writeln("    testThread received Variant: ", v);
            }
        );
    }
}

void main(){
    writeln("main thisTid: ", thisTid);
    Clock.currTime.writeln;
    
    auto testTid = spawn(&testThread);
    assert(testTid != thisTid);
    writeln("spawned ", testTid);
    
    addEvent(testTid, 1.seconds, 1);
    
    addEvent(thisTid, 2.seconds,                500, Yes.periodic);
    addEvent(thisTid, 2.seconds + 400.msecs,    600, Yes.periodic);
    
    addEvent(testTid, Clock.currTime + 2.seconds, "hello from main");
    
    while(true){
        receive(
            (int i){
                writeln("    main received int: ", i);
            },
            (string s){
                writeln("    main received string: ", s);
            },
            (DeleteEvent!string d){
                deleteEvent(thisTid, typeid(string), Delete.all);
            },
            (Variant v){
                writeln("    main received Variant: ", v);
            }
        );
    }
    
}
+/

void addEvent(T)(Tid receiver, SysTime time, T value){
    //writeln("  Adding event: ", Event(receiver, Variant(value), t, false, Duration.init));
    synchronized(events_lock){
        events ~= Event(receiver, Variant(value), time, false, Duration.init);
    }
    t.send(EventsModified());
}


void addEvent(T)(Tid receiver, Duration dt, T value, Flag!"periodic" periodic = No.periodic){
    //writeln("  Adding event: ", Event(receiver, Variant(value), Clock.currTime + dt, periodic, dt));
    synchronized(events_lock){
        events ~= Event(receiver, Variant(value), Clock.currTime + dt, periodic, dt);
    }
    t.send(EventsModified());
}

void deleteEvent(Tid receiver, TypeInfo type, Delete which){
    //writeln("  Deleting event: ", receiver, " ", type, " ", which);
    synchronized(events_lock){
        final switch(which) with(Delete){
        case all:
            events = events.remove!(a => a.receiver == receiver && a.value.type == type)();
            break;
        case first:
            auto idx = events.countUntil!(a => a.receiver == receiver && a.value.type == type);
            if(idx != -1){
                events = events.remove(idx);
            }
            break;
        case last:
            auto idx = events.length - 1 - events.retro.countUntil!(a => a.receiver == receiver && a.value.type == type);
            if(idx != -1){
                events = events.remove(idx);
            }
            break;
        }
    }
    t.send(EventsModified());
}

enum Delete {
    all,
    first,
    last
}




private:

shared static this(){
    events_lock = new Mutex;
    t = spawn(&proc);
}


struct Event {
    Tid         receiver;
    Variant     value;
    SysTime     triggerTime;
    bool        periodic;
    Duration    period;
}

struct EventsModified {}


__gshared Tid t;
__gshared Mutex events_lock;
__gshared Event[] events;



void proc(){
    Duration timeUntilNext = 1.hours;
    
    while(true){
        //writeln("Time until next: ", timeUntilNext);
        receiveTimeout( timeUntilNext,
            (EventsModified n){
            },
            (OwnerTerminated o){
            },
            (Variant v){
            }
        );
        timeUntilNext = 1.hours;
        synchronized(events_lock){
            events.sort!(q{a.triggerTime < b.triggerTime})();
            //events.map!(a => a.to!string ~ "\n").reduce!((a, b) => a ~ b).writeln;
            iter:
            foreach(idx, ref item; events){
                if(Clock.currTime >= item.triggerTime){
                    item.receiver.send(item.value);
                    if(item.periodic){
                        item.triggerTime += item.period;
                    } else {
                        events = events.remove(idx);
                        goto iter;
                    }
                }
            }
            auto now = Clock.currTime;
            timeUntilNext = events.length ? 
                max(events.map!(a => a.triggerTime - now).reduce!min, 0.seconds) : 
                1.hours;
        }
    }
}
















