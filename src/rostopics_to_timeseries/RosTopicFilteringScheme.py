from rostopics_to_timeseries.TopicMsgFilter import TopicMsgFilter

class RosTopicFilteringScheme(object):
    def __init__(self):
        pass
        self.filters = []

    def add_filter(self, topic_name, msg_type, filter_class):
        if not issubclass(filter_class, TopicMsgFilter):
            raise Exception("Filter to be added is not of subclass of TopicMsgFilter.")
        self.filters.append(
            (
                topic_name,
                msg_type,
                filter_class,
            )
        )

    def iter_filters(self):
        for i in self.filters:     
            yield i[0], i[1], i[2]
    
    @property
    def timeseries_header(self):
        names = []
        for tn, mt, fi in self.iter_filters():
            names.extend(fi.vector_meaning())
        return names

    @property
    def timeseries_size(self):
        size = 0
        for tn, mt, fi in self.iter_filters():
            size += fi.vector_size()
        return size

    @property
    def filter_amount(self):
        return len(self.filters)

    def _indent(self, diff=None):
        if not hasattr(self, '_indent_size'):
            self._indent_size = 0
        if diff == 0:
            self._indent_size = 0
        elif diff is not None:
            self._indent_size += 4*diff
        else: 
            return " "*self._indent_size
    @property
    def info(self):
        info = ""
        info += "filtering scheme info\n"
        self._indent(+1)
        for topic_name, msg_type, filter_class in self.iter_filters():
            info += self._indent()+topic_name+'\n'
            self._indent(+1)
            info += self._indent()+str(msg_type)+'\n'
            info += self._indent()+str(filter_class.vector_meaning())+'\n'
            self._indent(-1)
        self._indent(-1)
        info += "\n"
        return info
        


