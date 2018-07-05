from rostopics_to_timeseries.TopicMsgFilter import TopicMsgFilter
from rostopics_to_timeseries.Smoother import Smoother

class RosTopicFilteringScheme(object):
    def __init__(self, resampling_rate):
        pass
        self.filters = []
        if resampling_rate <= 0:
            raise Exception("Invalid resampling_rate: %s"%resampling_rate)
        self.rate = resampling_rate

    def add_filter(self, topic_name, msg_type, filter_class):
        if not issubclass(filter_class, TopicMsgFilter):
            raise Exception("Filter class to be added is not a subclass of TopicMsgFilter.")
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

        info += self._indent()+"resampling rate: %s\n"%str(self.rate) 
        info += '\n'

        info += self._indent()+"smoother_class: %s\n"%str(self.smoother_class) 
        info += '\n'

        info += self._indent()+"filters info:\n"
        self._indent(+1)
        for filter_no, (topic_name, msg_type, filter_class) in enumerate(self.iter_filters()):
            info += self._indent()+'filter No.%s: \n'%filter_no
            self._indent(+1)
            info += self._indent()+'topic_name: '+topic_name+'\n'
            info += self._indent()+'msg_type: '+str(msg_type)+'\n'
            info += self._indent()+'filter_class.vector_meaning: '+str(filter_class.vector_meaning())+'\n'
            self._indent(-1)
        self._indent(-1)


        info += "\n"
        return info

    @property
    def smoother_class(self):
        if hasattr(self, "smoother_class_"):
            return self.smoother_class_
        else:
            return None
    
    @smoother_class.setter
    def smoother_class(self, cls):
        if not issubclass(cls, Smoother) and cls is not None:
            raise Exception("Smoother class to be set is not a subclass of Smoother.")
        self.smoother_class_= cls


