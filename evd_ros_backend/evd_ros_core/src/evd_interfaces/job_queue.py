'''
'''

from functools import partial

from evd_ros_core.msg import Job
from evd_interfaces.frontend_interface import FrontendInterface


class JobQueue:

    def __init__(self, job_name, start_callback, end_callback, frontend=None):
        self._job_name = job_name
        self._pending_jobs = []
        self._active_job = None

        self._start_job = start_callback
        self._end_job = end_callback

        self._frontend = frontend if frontend != None else FrontendInterface()
        self._frontend.create_job_provider(self._job_name, self._handle_request_cb, self._handle_clear_cb, self._handle_query_cb)

    def _handle_request_cb(self, id, data):
        self.add(id, data)

    def _handle_clear_cb(self, id):
        self.cancel(id)

    def _handle_query_cb(self):
        aJob = Job(id=self._active_job['id'], data=json.dumps(self._active_job['data']))
        pJobs = [Job(id=p['id'], data=json.dumps(p['data'])) for p in self._pending_jobs]
        return aJob, pJobs

    @property
    def active_job(self):
        return self._active_job

    def add(self, id, data, allow_update_on_active_job=True):

        job = {
            'id': id,
            'data': data,
            'status': 'pending'
        }
        
        if self._active_job != None and id == self._active_job['id']:
            if not allow_update_on_active_job:
                raise Exception('Active job is being updated which is currently not allowed')
            
            self.cancel(id)
            self._pending_jobs.append(job)

        else: 
            existing = None
            for j in self._pending_jobs:
                if j['id'] == id:
                    existing = j
                    break

            if existing != None:
                existing['data'] = data
            else:
                self._pending_jobs.append(job)
        
    def cancel(self, id):
        if self._active_job != None and id == self._active_job['id']:
            self._active_job['status'] = 'canceled'
        else:
            existing = None
            for j in self._pending_jobs:
                if j['id'] == id:
                    existing = j
                    break

            if existing != None:
                self._pending_jobs.remove(existing)

    def update(self):
        
        # Handle active job is completed
        if self._active_job != None:
            submit_fnt = partial(self._frontend.submit_job, self._job_name, self._active_job['id'])

            if self._active_job['status'] == 'done':
                self._end_job(True,submit_fnt)
                self._active_job = None
            elif self._active_job['status'] == 'canceled':
                self._end_job(False,submit_fnt)
                self._active_job = None
        
        # try to get a next job
        if self._active_job == None:
            if len(self._pending_jobs) > 0:
                self._active_job = self._pending_jobs.pop(0)
                self._start_job(self._active_job['data'])

    def completed(self):
        self._active_job['status'] = 'done'