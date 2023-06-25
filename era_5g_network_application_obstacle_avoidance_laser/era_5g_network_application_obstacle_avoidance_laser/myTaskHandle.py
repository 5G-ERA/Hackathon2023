from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ
from queue import Full, Queue

class customTaskHandler(TaskHandlerInternalQ):
    def __init__(self,sid: str, scan_width: int, image_queue: Queue, **kw) -> None:
        super().__init__(sid, image_queue, **kw)
        self.scan_width = scan_width
        
    def store_laser(self, metadata: dict, laser: dict):
        try:
            self._q.put((metadata, laser), block=False)
        except Full:
            print('error')
            pass
